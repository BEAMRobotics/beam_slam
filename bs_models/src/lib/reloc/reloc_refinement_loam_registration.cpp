#include <bs_models/reloc/reloc_refinement_loam_registration.h>

#include <nlohmann/json.hpp>

#include <beam_utils/filesystem.h>

#include <bs_common/conversions.h>
#include <bs_common/utils.h>

namespace bs_models::reloc {

using namespace global_mapping;
using namespace beam_matching;

RelocRefinementLoam::RelocRefinementLoam(const std::string& config)
    : RelocRefinementBase(config) {
  LoadConfig();
  Setup();
}

RelocRefinementResults RelocRefinementLoam::RunRefinement(
    const global_mapping::SubmapPtr& matched_submap,
    const global_mapping::SubmapPtr& query_submap,
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
  RelocRefinementResults results;
  Eigen::Matrix<double, 6, 6> covariance;
  if (!GetRefinedT_SUBMAP_QUERY(matched_submap_in_submap_frame,
                                query_submap_in_submap_frame, T_MATCH_QUERY_EST,
                                results.T_MATCH_QUERY, covariance)) {
    return {};
  }
  results.successful = true;
  results.covariance = covariance;
  return results;
}

void RelocRefinementLoam::LoadConfig() {
  if (config_path_.empty()) {
    BEAM_INFO("No config file provided to RelocRefinementLoam, using "
              "default parameters.");
    return;
  }

  BEAM_INFO("Loading reloc config: {}", config_path_);
  nlohmann::json J;
  if (!beam::ReadJson(config_path_, J)) {
    BEAM_ERROR("Unable to read config");
    throw std::runtime_error{"Unable to read config"};
  }

  bs_common::ValidateJsonKeysOrThrow(std::vector<std::string>{"matcher_config"},
                                     J);
  std::string matcher_config_rel = J["matcher_config"];
  if (matcher_config_rel.empty()) {
    BEAM_ERROR("Reloc refinement cannot have an empty matcher_config");
    throw std::runtime_error{"invalid json inputs"};
  }
  matcher_config_ = beam::CombinePaths(bs_common::GetBeamSlamConfigPath(),
                                       matcher_config_rel);
}

void RelocRefinementLoam::Setup() {
  // load matcher
  beam_matching::LoamParams matcher_params(matcher_config_);
  matcher_ = std::make_unique<LoamMatcher>(matcher_params);
}

bool RelocRefinementLoam::GetRefinedT_SUBMAP_QUERY(
    const LoamPointCloud& submap_cloud, const LoamPointCloud& query_cloud,
    const Eigen::Matrix4d& T_SUBMAP_QUERY_EST,
    Eigen::Matrix4d& T_SUBMAP_QUERY_OPT,
    Eigen::Matrix<double, 6, 6>& covariance) {
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
  covariance = matcher_->GetCovariance();

  return true;
}

} // namespace bs_models::reloc
