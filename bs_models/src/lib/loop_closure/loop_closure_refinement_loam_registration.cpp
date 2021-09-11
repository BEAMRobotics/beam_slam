#include <bs_models/loop_closure/loop_closure_refinement_loam_registration.h>

#include <nlohmann/json.hpp>

#include <beam_utils/filesystem.h>

#include <bs_common/utils.h>

namespace bs_models {

namespace loop_closure {

LoopClosureRefinementLoam::LoopClosureRefinementLoam(
    const Eigen::Matrix<double, 6, 6>& loop_closure_covariance,
    const std::string& config)
    : LoopClosureRefinementBase(config),
      loop_closure_covariance_(loop_closure_covariance) {
  LoadConfig();
  Setup();
}

fuse_core::Transaction::SharedPtr
LoopClosureRefinementLoam::GenerateTransaction(
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
  transaction.AddPoseConstraint(
      matched_submap->T_WORLD_SUBMAP(), query_submap->T_WORLD_SUBMAP(),
      matched_submap->Stamp(), query_submap->Stamp(), T_MATCH_QUERY_OPT,
      loop_closure_covariance_, source_);

  return transaction.GetTransaction();
}

void LoopClosureRefinementLoam::LoadConfig() {
  std::string read_path = config_path_;
  
  if (read_path.empty()) {
    return;
  }

  if (read_path == "DEFAULT_PATH") {
    read_path = bs_common::GetBeamSlamConfigPath() +
                "global_map/loop_closure_refinement_loam_registration.json";
  }

  BEAM_INFO("Loading loop closure config: {}", read_path);
  nlohmann::json J;
  if (!beam::ReadJson(read_path, J)) {
    BEAM_INFO("Using default params.");
    return;
  }

  try {
    matcher_config_ = J["matcher_config"];
  } catch (...) {
    BEAM_ERROR(
        "Missing one or more parameter, using default loop closure params.");
  }
}

void LoopClosureRefinementLoam::Setup() {
  // load matcher
  beam_matching::LoamParams matcher_params(matcher_config_);
  matcher_ = std::make_unique<beam_matching::LoamMatcher>(matcher_params);
}

bool LoopClosureRefinementLoam::GetRefinedT_MATCH_QUERY(
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
    BEAM_WARN("Failed scan matching. Not adding loop closure constraint.");
    return false;
  }

  T_MATCH_QUERY_OPT = matcher_->GetResult().inverse().matrix();
}

}  // namespace loop_closure

}  // namespace bs_models
