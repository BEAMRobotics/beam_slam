#include <global_mapping/loop_closure/loop_closure_refinement_loam_registration.h>

#include <nlohmann/json.hpp>

namespace global_mapping {

LoopClosureRefinementLoam::LoopClosureRefinementLoam(
    const Eigen::Matrix<double, 6, 6>& loop_closure_covariance,
    const std::string& matcher_config)
    : loop_closure_covariance_(loop_closure_covariance),
      matcher_config_(matcher_config) {
  covariance_set_ = true;
}

fuse_core::Transaction::SharedPtr
LoopClosureRefinementLoam::GenerateTransaction(
    const Submap& matched_submap, const Submap& query_submap,
    const Eigen::Matrix4d& T_MATCH_QUERY_EST) {
  LoadConfig();
  Setup();

  Eigen::Matrix4d T_MATCH_QUERY_OPT;
  if (!GetRefinedT_MATCH_QUERY(matched_submap, query_submap, T_MATCH_QUERY_EST,
                               T_MATCH_QUERY_OPT)) {
    return nullptr;
  }

  std::string source = "LOOPCLOSURE";
  bs_constraints::frame_to_frame::Pose3DStampedTransaction transaction(
      query_submap.Stamp());
  transaction.AddPoseConstraint(
      matched_submap.T_WORLD_SUBMAP(), query_submap.T_WORLD_SUBMAP(),
      matched_submap.Stamp(), query_submap.Stamp(), T_MATCH_QUERY_OPT,
      loop_closure_covariance_, source);

  return transaction.GetTransaction();
}

void LoopClosureRefinementLoam::LoadConfig() {
  if (config_path_.empty()) {
    return;
  }

  if (!boost::filesystem::exists(config_path_)) {
    BEAM_ERROR(
        "Invalid path to loop closure candidate search config. Using default "
        "parameters. Input: {}",
        config_path_);
    return;
  }

  nlohmann::json J;
  std::ifstream file(config_path_);
  file >> J;
  matcher_config_ = J["matcher_config_path"];
}

bool LoopClosureRefinementLoam::GetRefinedT_MATCH_QUERY(
    const Submap& matched_submap, const Submap& query_submap,
    const Eigen::Matrix4d& T_MATCH_QUERY_EST,
    Eigen::Matrix4d& T_MATCH_QUERY_OPT) {
  LoamPointCloudPtr cloud_match_world = std::make_shared<LoamPointCloud>(
      matched_submap.GetLidarLoamPointsInWorldFrame());
  LoamPointCloudPtr cloud_query_world = std::make_shared<LoamPointCloud>(
      query_submap.GetLidarLoamPointsInWorldFrame());
  matcher_->SetRef(cloud_match_world);
  matcher_->SetTarget(cloud_query_world);
  if (!matcher_->Match()) {
    BEAM_WARN("Failed scan matching. Not adding loop closure constraint.");
    return false;
  }
  T_MATCH_QUERY_OPT = matcher_->GetResult().inverse().matrix();
}

}  // namespace global_mapping