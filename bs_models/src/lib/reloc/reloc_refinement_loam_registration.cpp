#include <bs_models/reloc/reloc_refinement_loam_registration.h>

#include <nlohmann/json.hpp>

#include <beam_utils/filesystem.h>

#include <bs_common/utils.h>

namespace bs_models {

namespace reloc {

RelocRefinementLoam::RelocRefinementLoam(
    const Eigen::Matrix<double, 6, 6>& reloc_covariance,
    const std::string& config)
    : RelocRefinementBase(config), reloc_covariance_(reloc_covariance) {
  LoadConfig();
  Setup();
}

fuse_core::Transaction::SharedPtr RelocRefinementLoam::GenerateTransaction(
    const std::shared_ptr<global_mapping::Submap>& matched_submap,
    const std::shared_ptr<global_mapping::Submap>& query_submap,
    const Eigen::Matrix4d& T_MATCH_QUERY_EST) {
  // get refined transform
  Eigen::Matrix4d T_MATCH_QUERY_OPT;
  if (!GetRefinedT_MATCH_QUERY(matched_submap, query_submap, T_MATCH_QUERY_EST,
                               T_MATCH_QUERY_OPT)) {
    return nullptr;
  }

  // create transaction
  bs_constraints::relative_pose::Pose3DStampedTransaction transaction(
      query_submap->Stamp());
  transaction.AddPoseConstraint(matched_submap->T_WORLD_SUBMAP(),
                                query_submap->T_WORLD_SUBMAP(),
                                matched_submap->Stamp(), query_submap->Stamp(),
                                T_MATCH_QUERY_OPT, reloc_covariance_, source_);

  return transaction.GetTransaction();
}

bool RelocRefinementLoam::GetRefinedPose(
    Eigen::Matrix4d& T_SUBMAP_QUERY_refined,
    const Eigen::Matrix4d& T_SUBMAP_QUERY_initial,
    const std::shared_ptr<global_mapping::Submap>& submap,
    const PointCloud& lidar_cloud_in_query_frame,
    const cv::Mat& image) {
  // TODO
  BEAM_ERROR("Not yet implemented");
}

void RelocRefinementLoam::LoadConfig() {
  std::string read_path = config_path_;

  if (read_path.empty()) {
    return;
  }

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
  beam_matching::LoamParams matcher_params(matcher_config_);
  matcher_ = std::make_unique<beam_matching::LoamMatcher>(matcher_params);
}

bool RelocRefinementLoam::GetRefinedT_MATCH_QUERY(
    const std::shared_ptr<global_mapping::Submap>& matched_submap,
    const std::shared_ptr<global_mapping::Submap>& query_submap,
    const Eigen::Matrix4d& T_MATCH_QUERY_EST,
    Eigen::Matrix4d& T_MATCH_QUERY_OPT) {
  beam_matching::LoamPointCloudPtr cloud_match_world =
      std::make_shared<beam_matching::LoamPointCloud>(
          matched_submap->GetLidarLoamPointsInWorldFrame());
  beam_matching::LoamPointCloudPtr cloud_query_world =
      std::make_shared<beam_matching::LoamPointCloud>(
          query_submap->GetLidarLoamPointsInWorldFrame());

  matcher_->SetRef(cloud_match_world);
  matcher_->SetTarget(cloud_query_world);
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

  Eigen::Matrix4d T_MATCHREFINED_MATCHEST =
      matcher_->GetResult().inverse().matrix();

  /**
   * Get refined pose:
   * T_MATCH_QUERY_OPT = T_MATCHREFINED_QUERY
   *                   = T_MATCHREFINED_MATCHEST * T_MATCHEST_QUERY
   *                   = T_MATCHREFINED_MATCHEST * T_MATCH_QUERY_EST
   */
  T_MATCH_QUERY_OPT = T_MATCHREFINED_MATCHEST * T_MATCH_QUERY_EST;

  if (output_results_) {
    output_path_stamped_ =
        debug_output_path_ +
        beam::ConvertTimeToDate(std::chrono::system_clock::now()) + "_passed/";
    boost::filesystem::create_directory(output_path_stamped_);
    matcher_->SaveResults(output_path_stamped_);
  }

  return true;
}

}  // namespace reloc

}  // namespace bs_models
