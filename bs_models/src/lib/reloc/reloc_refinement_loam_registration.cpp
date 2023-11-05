#include <bs_models/reloc/reloc_refinement_loam_registration.h>

#include <filesystem>

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
    const Eigen::Matrix4d& T_MATCH_QUERY_EST, const std::string& output_path) {
  // extract and filter clouds from matched submap
  LoamPointCloud matched_submap_in_submap_frame(
      matched_submap->GetLidarLoamPointsInWorldFrame(),
      beam::InvertTransform(matched_submap->T_WORLD_SUBMAP()));

  // extract and filter clouds from matched submap
  LoamPointCloud query_submap_in_submap_frame(
      query_submap->GetLidarLoamPointsInWorldFrame(),
      beam::InvertTransform(query_submap->T_WORLD_SUBMAP()));

  // create output path
  std::string current_output_path;
  if (!output_path.empty()) {
    if (!std::filesystem::exists(output_path)) {
      BEAM_ERROR("Output path for RelocRefinement does not exist: {}",
                 output_path);
      throw std::runtime_error{"invalid output path"};
    }
    std::string query_stamp = std::to_string(query_submap->Stamp().toSec());
    current_output_path = beam::CombinePaths(output_path, query_stamp);

    BEAM_INFO("Saving reloc refinement results to: {}", current_output_path);
    std::filesystem::create_directory(current_output_path);
  }

  // get refined transform
  RelocRefinementResults results;
  Eigen::Matrix<double, 6, 6> covariance;
  results.successful = GetRefinedT_SUBMAP_QUERY(
      matched_submap_in_submap_frame, query_submap_in_submap_frame,
      T_MATCH_QUERY_EST, current_output_path, results.T_MATCH_QUERY,
      covariance);
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

  beam::ValidateJsonKeysOrThrow(std::vector<std::string>{"matcher_config"}, J);
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
  std::string ceres_config =
      bs_common::GetAbsoluteConfigPathFromJson(matcher_config_, "ceres_config");
  beam_matching::LoamParams matcher_params(matcher_config_, ceres_config);
  matcher_ = std::make_unique<LoamMatcher>(matcher_params);
}

bool RelocRefinementLoam::GetRefinedT_SUBMAP_QUERY(
    const LoamPointCloud& submap_cloud, const LoamPointCloud& query_cloud,
    const Eigen::Matrix4d& T_SUBMAP_QUERY_EST, const std::string& output_path,
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
    if (!output_path.empty()) {
      matcher_->SaveResults(output_path, "failed_cloud_");
    }
    return false;
  }

  T_SUBMAP_QUERY_OPT = matcher_->ApplyResult(T_SUBMAP_QUERY_EST);

  if (!output_path.empty()) { matcher_->SaveResults(output_path, "cloud_"); }
  covariance = matcher_->GetCovariance();

  return true;
}

} // namespace bs_models::reloc
