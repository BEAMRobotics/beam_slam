#include <bs_models/reloc/reloc_refinement_loam_registration.h>

#include <nlohmann/json.hpp>

#include <beam_utils/filesystem.h>

#include <bs_common/utils.h>
#include <bs_common/conversions.h>

namespace bs_models { namespace reloc {

RelocRefinementLoam::RelocRefinementLoam(
    const Eigen::Matrix<double, 6, 6>& reloc_covariance,
    const std::string& config)
    : RelocRefinementBase(config), reloc_covariance_(reloc_covariance) {
  LoadConfig();
  Setup();
}

fuse_core::Transaction::SharedPtr RelocRefinementLoam::GenerateTransaction(
    const SubmapPtr& matched_submap, const SubmapPtr& query_submap,
    const Eigen::Matrix4d& T_MATCH_QUERY_EST) {
  // extract and filter clouds from matched submap
  LoamPointCloud matched_submap_in_submap_frame(
      matched_submap->GetLidarLoamPointsInWorldFrame(),
      beam::InvertTransform(matched_submap->T_WORLD_SUBMAP()));

  // extract and filter clouds from matched submap
  LoamPointCloud query_submap_in_submap_frame(
      query_submap->GetLidarLoamPointsInWorldFrame(),
      beam::InvertTransform(query_submap->T_WORLD_SUBMAP()));

  // get refined transform
  Eigen::Matrix4d T_MATCH_QUERY_OPT;
  if (!GetRefinedT_SUBMAP_QUERY(matched_submap_in_submap_frame,
                                query_submap_in_submap_frame, T_MATCH_QUERY_EST,
                                T_MATCH_QUERY_OPT)) {
    return nullptr;
  }

  // create transaction
  bs_constraints::Pose3DStampedTransaction transaction(query_submap->Stamp());
  fuse_variables::Position3DStamped p_diff(query_submap->Stamp());
  fuse_variables::Orientation3DStamped o_diff(query_submap->Stamp());
  bs_common::EigenTransformToFusePose(T_MATCH_QUERY_OPT, p_diff, o_diff);
  transaction.AddPoseConstraint(
      matched_submap->Position(), query_submap->Position(),
      matched_submap->Orientation(), query_submap->Orientation(), p_diff,
      o_diff, reloc_covariance_, source_);

  return transaction.GetTransaction();
}

bool RelocRefinementLoam::GetRefinedPose(
    Eigen::Matrix4d& T_SUBMAP_QUERY_refined,
    const Eigen::Matrix4d& T_SUBMAP_QUERY_initial, const SubmapPtr& submap,
    const PointCloud& lidar_cloud_in_query_frame,
    const LoamPointCloudPtr& loam_cloud_in_query_frame, const cv::Mat& image) {
  // extract and filter clouds from match submap
  LoamPointCloud submap_in_submap_frame(
      submap->GetLidarLoamPointsInWorldFrame(),
      beam::InvertTransform(submap->T_WORLD_SUBMAP()));

  // get refined transform
  if (!GetRefinedT_SUBMAP_QUERY(
          submap_in_submap_frame, *loam_cloud_in_query_frame,
          T_SUBMAP_QUERY_initial, T_SUBMAP_QUERY_refined)) {
    return false;
  }

  return true;
}

void RelocRefinementLoam::LoadConfig() {
  std::string read_path = config_path_;

  if (read_path.empty()) { return; }

  if (read_path == "DEFAULT_PATH") {
    read_path = bs_common::GetBeamSlamConfigPath() +
                "global_map/reloc_refinement_loam_registration.json";
  }

  BEAM_INFO("Loading reloc config: {}", read_path);
  nlohmann::json J;
  if (!beam::ReadJson(read_path, J)) {
    BEAM_INFO("Using default params.");
    return;
  }

  try {
    matcher_config_ = J["matcher_config"];
  } catch (...) {
    BEAM_ERROR("Missing one or more parameter, using default reloc params.");
  }
}

void RelocRefinementLoam::Setup() {
  // load matcher
  LoamParams matcher_params(matcher_config_);
  matcher_ = std::make_unique<LoamMatcher>(matcher_params);
}

bool RelocRefinementLoam::GetRefinedT_SUBMAP_QUERY(
    const LoamPointCloud& submap_cloud, const LoamPointCloud& query_cloud,
    const Eigen::Matrix4d& T_SUBMAP_QUERY_EST,
    Eigen::Matrix4d& T_SUBMAP_QUERY_OPT) {
  // convert query to estimated submap frame and make pointers
  LoamPointCloudPtr submap_ptr = std::make_shared<LoamPointCloud>(submap_cloud);
  LoamPointCloudPtr query_in_submap_frame_est =
      std::make_shared<LoamPointCloud>(query_cloud);
  query_in_submap_frame_est->TransformPointCloud(T_SUBMAP_QUERY_EST);

  // match clouds
  matcher_->SetRef(submap_ptr);
  matcher_->SetTarget(query_in_submap_frame_est);

  if (!matcher_->Match()) {
    BEAM_WARN("Failed scan matching. Not adding reloc constraint.");
    if (output_results_) {
      output_path_stamped_ =
          debug_output_path_ +
          beam::ConvertTimeToDate(std::chrono::system_clock::now()) +
          "_failed/";
      boost::filesystem::create_directory(output_path_stamped_);
      matcher_->SaveResults(output_path_stamped_);
    }
    return false;
  }

  Eigen::Matrix4d T_SUBMAPREFINED_SUBMAPEST =
      matcher_->GetResult().inverse().matrix();

  /**
   * Get refined pose:
   * T_SUBMAP_QUERY_OPT = T_SUBMAPREFINED_QUERY
   *                   = T_SUBMAPREFINED_SUBMAPEST * T_SUBMAPEST_QUERY
   *                   = T_SUBMAPREFINED_SUBMAPEST * T_SUBMAP_QUERY_EST
   */
  T_SUBMAP_QUERY_OPT = T_SUBMAPREFINED_SUBMAPEST * T_SUBMAP_QUERY_EST;

  if (output_results_) {
    output_path_stamped_ =
        debug_output_path_ +
        beam::ConvertTimeToDate(std::chrono::system_clock::now()) + "_passed/";
    boost::filesystem::create_directory(output_path_stamped_);
    matcher_->SaveResults(output_path_stamped_);
  }

  return true;
}

}} // namespace bs_models::reloc
