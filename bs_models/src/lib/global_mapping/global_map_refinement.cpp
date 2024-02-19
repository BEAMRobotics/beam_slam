#include <bs_models/global_mapping/global_map_refinement.h>

#include <filesystem>

namespace bs_models::global_mapping {

void GlobalMapRefinement::Params::LoadJson(const std::string& config_path) {
  // Read json
  if (config_path.empty()) {
    BEAM_INFO("No config file provided to global map refinement, using default "
              "parameters.");
    return;
  }
  BEAM_INFO("Loading global map refinement config file: {}", config_path);
  nlohmann::json J;
  if (!beam::ReadJson(config_path, J)) {
    BEAM_ERROR("Unable to read global map refinement config");
    throw std::runtime_error{"Unable to read global map refinement config"};
  }

  beam::ValidateJsonKeysOrThrow({"loop_closure", "submap_refinement",
                                 "submap_alignment", "batch_optimizer"},
                                J);

  // load loop closure params
  nlohmann::json J_loop_closure = J["loop_closure"];
  beam::ValidateJsonKeysOrThrow({"candidate_search_config", "refinement_config",
                                 "local_mapper_covariance",
                                 "loop_closure_covariance"},
                                J_loop_closure);

  std::string candidate_search_config_rel =
      J_loop_closure["candidate_search_config"];
  if (!candidate_search_config_rel.empty()) {
    submap_pgo.candidate_search_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), candidate_search_config_rel);
  }

  double lc_cov_dia = J_loop_closure["loop_closure_covariance"];
  double lm_cov_dia = J_loop_closure["local_mapper_covariance"];
  submap_pgo.loop_closure_covariance =
      Eigen::Matrix<double, 6, 6>::Identity() * lc_cov_dia;
  submap_pgo.local_mapper_covariance =
      Eigen::Matrix<double, 6, 6>::Identity() * lm_cov_dia;

  std::string refinement_config_rel = J_loop_closure["refinement_config"];
  if (!refinement_config_rel.empty()) {
    submap_pgo.refinement_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), refinement_config_rel);
  }

  // load submap refinement params
  nlohmann::json J_submap_refinement = J["submap_refinement"];
  beam::ValidateJsonKeysOrThrow({"scan_registration_config", "matcher_config"},
                                J_submap_refinement);

  std::string scan_registration_config_rel =
      J_submap_refinement["scan_registration_config"];
  if (!scan_registration_config_rel.empty()) {
    submap_refinement.scan_registration_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), scan_registration_config_rel);
  }

  std::string matcher_config_rel = J_submap_refinement["matcher_config"];
  if (!matcher_config_rel.empty()) {
    submap_refinement.matcher_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), matcher_config_rel);
  }

  // load submap alignment params
  nlohmann::json J_submap_alignment = J["submap_alignment"];
  beam::ValidateJsonKeysOrThrow({"matcher_config"}, J_submap_alignment);
  matcher_config_rel = J_submap_alignment["matcher_config"];
  if (!matcher_config_rel.empty()) {
    submap_alignment.matcher_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), matcher_config_rel);
  }

  // load batch optimizer params
  nlohmann::json J_batch_optimizer = J["batch_optimizer"];
  beam::ValidateJsonKeysOrThrow({"matcher_config", "scan_registration_config",
                                 "update_graph_on_all_scans",
                                 "lc_dist_thresh_m", "lc_min_traj_dist_m",
                                 "lc_scan_context_dist_thres"},
                                J_batch_optimizer);
  matcher_config_rel = J_batch_optimizer["matcher_config"];
  if (!matcher_config_rel.empty()) {
    batch.matcher_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), matcher_config_rel);
  }
  scan_registration_config_rel = J_batch_optimizer["scan_registration_config"];
  if (!scan_registration_config_rel.empty()) {
    batch.scan_registration_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), scan_registration_config_rel);
  }
  batch.update_graph_on_all_scans =
      J_batch_optimizer["update_graph_on_all_scans"];
  batch.lc_dist_thresh_m = J_batch_optimizer["lc_dist_thresh_m"];
  batch.lc_min_traj_dist_m = J_batch_optimizer["lc_min_traj_dist_m"];
  batch.lc_scan_context_dist_thres =
      J_batch_optimizer["lc_scan_context_dist_thres"];
}

void GlobalMapRefinement::Summary::Save(const std::string& output_path) const {
  nlohmann::json J;
  std::vector<nlohmann::json> J_submap_refinement;
  for (const auto& [stamp, result] : submap_refinement) {
    nlohmann::json J_result;
    J_result["dt_mm"] = result.dt;
    J_result["dR_deg"] = result.dR;
    J_result["sec"] = stamp.sec;
    J_result["nsec"] = stamp.nsec;
    J_submap_refinement.push_back(J_result);
  }
  J["submap_refinement"] = J_submap_refinement;

  std::vector<nlohmann::json> J_submap_alignment;
  for (const auto& [stamp, result] : submap_alignment) {
    nlohmann::json J_result;
    J_result["dt_mm"] = result.dt;
    J_result["dR_deg"] = result.dR;
    J_result["sec"] = stamp.sec;
    J_result["nsec"] = stamp.nsec;
    J_submap_alignment.push_back(J_result);
  }
  J["submap_alignment"] = J_submap_alignment;

  std::string summary_path = beam::CombinePaths(output_path, "summary.json");
  std::ofstream file(summary_path);
  file << std::setw(4) << J << std::endl;
}

GlobalMapRefinement::GlobalMapRefinement(const std::string& global_map_data_dir,
                                         const Params& params)
    : params_(params) {
  BEAM_INFO("Loading global map data from: {}", global_map_data_dir);
  global_map_ = std::make_shared<GlobalMap>(global_map_data_dir);
}

GlobalMapRefinement::GlobalMapRefinement(const std::string& global_map_data_dir,
                                         const std::string& config_path) {
  params_.LoadJson(config_path);
  BEAM_INFO("Loading global map data from: {}", global_map_data_dir);
  global_map_ = std::make_shared<GlobalMap>(global_map_data_dir);
}

GlobalMapRefinement::GlobalMapRefinement(std::shared_ptr<GlobalMap>& global_map,
                                         const Params& params)
    : global_map_(global_map), params_(params) {}

GlobalMapRefinement::GlobalMapRefinement(std::shared_ptr<GlobalMap>& global_map,
                                         const std::string& config_path)
    : global_map_(global_map) {
  params_.LoadJson(config_path);
}

bool GlobalMapRefinement::RunSubmapRefinement(const std::string& output_path) {
  if (!output_path.empty()) {
    global_map_->SaveTrajectoryClouds(output_path, false);
    std::filesystem::rename(
        beam::CombinePaths(output_path, "global_map_trajectory_optimized.pcd"),
        beam::CombinePaths(output_path, "trajectory_initial.pcd"));
  }

  std::vector<SubmapPtr> submaps = global_map_->GetSubmaps();
  SubmapRefinement refinement(params_.submap_refinement, output_path);
  refinement.Run(submaps);
  summary_.submap_alignment = refinement.GetResults();

  if (!output_path.empty()) {
    global_map_->SaveTrajectoryClouds(output_path, false);
    std::filesystem::rename(
        beam::CombinePaths(output_path, "global_map_trajectory_optimized.pcd"),
        beam::CombinePaths(output_path, "trajectory_final.pcd"));
  }
  return true;
}

bool GlobalMapRefinement::RunSubmapAlignment(const std::string& output_path) {
  std::vector<SubmapPtr> submaps = global_map_->GetSubmaps();
  if (submaps.size() < 2) {
    BEAM_WARN(
        "Not enough submaps to run submap alignment, at least two are needed");
    return true;
  }

  if (!output_path.empty()) {
    global_map_->SaveTrajectoryClouds(output_path, false);
    std::filesystem::rename(
        beam::CombinePaths(output_path, "global_map_trajectory_optimized.pcd"),
        beam::CombinePaths(output_path, "trajectory_initial.pcd"));
  }

  SubmapAlignment alignment(params_.submap_alignment, output_path);
  alignment.Run(submaps);
  summary_.submap_alignment = alignment.GetResults();

  if (!output_path.empty()) {
    global_map_->SaveTrajectoryClouds(output_path, false);
    std::filesystem::rename(
        beam::CombinePaths(output_path, "global_map_trajectory_optimized.pcd"),
        beam::CombinePaths(output_path, "trajectory_final.pcd"));
  }

  return true;
}

bool GlobalMapRefinement::RunPoseGraphOptimization(
    const std::string& output_path) {
  if (!output_path.empty()) {
    global_map_->SaveTrajectoryClouds(output_path, false);
    std::filesystem::rename(
        beam::CombinePaths(output_path, "global_map_trajectory_optimized.pcd"),
        beam::CombinePaths(output_path, "trajectory_initial.pcd"));
  }

  std::vector<SubmapPtr> submaps = global_map_->GetSubmaps();
  SubmapPoseGraphOptimization pgo(params_.submap_pgo, output_path);
  pgo.Run(submaps);

  if (!output_path.empty()) {
    global_map_->SaveTrajectoryClouds(output_path, false);
    std::filesystem::rename(
        beam::CombinePaths(output_path, "global_map_trajectory_optimized.pcd"),
        beam::CombinePaths(output_path, "trajectory_final.pcd"));
  }

  return true;
}

bool GlobalMapRefinement::RunBatchOptimization(const std::string& output_path) {
  if (!output_path.empty()) {
    global_map_->SaveTrajectoryClouds(output_path, false);
    std::filesystem::rename(
        beam::CombinePaths(output_path, "global_map_trajectory_optimized.pcd"),
        beam::CombinePaths(output_path, "trajectory_initial.pcd"));
  }

  std::vector<SubmapPtr> submaps = global_map_->GetSubmaps();
  GlobalMapBatchOptimization batch(params_.batch, output_path);
  batch.Run(submaps);

  // save final trajectory
  if (!output_path.empty()) {
    global_map_->SaveTrajectoryClouds(output_path, false);
    std::filesystem::rename(
        beam::CombinePaths(output_path, "global_map_trajectory_optimized.pcd"),
        beam::CombinePaths(output_path, "trajectory_final.pcd"));
  }

  return true;
}

void GlobalMapRefinement::SaveResults(const std::string& output_path,
                                      bool save_initial) {
  // create results directory
  if (!boost::filesystem::exists(output_path)) {
    BEAM_ERROR("Output directory does not exist, not outputting global map "
               "refinement "
               "results. Input: {}",
               output_path);
    return;
  }

  // save
  summary_.Save(output_path);
  global_map_->SaveTrajectoryFile(output_path, save_initial);
  global_map_->SaveTrajectoryClouds(output_path, save_initial);
  global_map_->SaveSubmapFrames(output_path, save_initial);
  global_map_->SaveLidarSubmaps(output_path, save_initial);
  global_map_->SaveKeypointSubmaps(output_path, save_initial);
}

void GlobalMapRefinement::SaveGlobalMapData(const std::string& output_path) {
  // create results directory
  if (!boost::filesystem::exists(output_path)) {
    BEAM_ERROR("Output directory does not exist, not outputting global map "
               "refinement "
               "results. Input: {}",
               output_path);
    return;
  }

  std::string save_dir =
      beam::CombinePaths(output_path, "global_map_data_refined");
  boost::filesystem::create_directory(save_dir);

  // save
  global_map_->SaveData(save_dir);
}

} // namespace bs_models::global_mapping