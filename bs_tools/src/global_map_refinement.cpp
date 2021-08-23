#include <bs_tools/global_map_refinement.h>

#include <fuse_core/transaction.h>
#include <fuse_graphs/hash_graph.h>

#include <bs_common/utils.h>
#include <bs_models/global_mapping/loop_closure/loop_closure_methods.h>

namespace bs_tools {

GlobalMapRefinement::Params::Params() {
  double scan_reg_cov_diag = 1e-3;
  double loop_cov_diag = 1e-5;

  // clang-format off
  scan_reg_covariance << scan_reg_cov_diag, 0, 0, 0, 0, 0,
                             0, scan_reg_cov_diag, 0, 0, 0, 0,
                             0, 0, scan_reg_cov_diag, 0, 0, 0,
                             0, 0, 0, scan_reg_cov_diag, 0, 0,
                             0, 0, 0, 0, scan_reg_cov_diag, 0,
                             0, 0, 0, 0, 0, scan_reg_cov_diag;

  loop_closure_covariance << loop_cov_diag, 0, 0, 0, 0, 0,
                             0, loop_cov_diag, 0, 0, 0, 0,
                             0, 0, loop_cov_diag, 0, 0, 0,
                             0, 0, 0, loop_cov_diag, 0, 0,
                             0, 0, 0, 0, loop_cov_diag, 0,
                             0, 0, 0, 0, 0, loop_cov_diag;
  // clang-format on

  multi_scan_reg_params.outlier_threshold_t = 0.1;
  multi_scan_reg_params.outlier_threshold_r = 20;
  multi_scan_reg_params.min_motion_trans_m = 0;
  multi_scan_reg_params.min_motion_rot_rad = 0;
  multi_scan_reg_params.max_motion_trans_m = 5;
  multi_scan_reg_params.source = "MULTISCANREGISTRATION";
  multi_scan_reg_params.num_neighbors = 10;
  multi_scan_reg_params.disable_lidar_map = true;  // don't need

  // set this high because we don't want to remove any scans due to lag duration
  // overflow (this is offline)
  multi_scan_reg_params.lag_duration = 100000;

  // TODO: Do we want this to always be true? What about when we have vision?
  multi_scan_reg_params.fix_first_scan = true;

  scan_to_map_reg_params.outlier_threshold_t = 0.1;
  scan_to_map_reg_params.outlier_threshold_r = 20;
  scan_to_map_reg_params.min_motion_trans_m = 0;
  scan_to_map_reg_params.min_motion_rot_rad = 0;
  scan_to_map_reg_params.max_motion_trans_m = 5;
  scan_to_map_reg_params.source = "SCANTOMAPREGISTRATION";
  scan_to_map_reg_params.map_size = 20;
  scan_to_map_reg_params.store_full_cloud = false;  // don't need

  // TODO: Do we want this to always be true? What about when we have vision?
  scan_to_map_reg_params.fix_first_scan = true;

  loam_matcher_params.max_correspondence_distance = 0.3;
  loam_matcher_params.validate_correspondences = false;
  loam_matcher_params.iterate_correspondences = true;
  loam_matcher_params.convergence_criteria_translation_m = 0.001;
  loam_matcher_params.convergence_criteria_rotation_deg = 0.1;
  loam_matcher_params.max_correspondence_iterations = 5;
  loam_matcher_params.output_ceres_summary = false;
}

void GlobalMapRefinement::Params::LoadJson(const std::string& config_path) {
  std::string read_file = config_path;
  if (read_file.empty()) {
    BEAM_INFO(
        "No config file provided to global map refinement, using default "
        "parameters.");
    return;
  }

  if (read_file == "DEFAULT_PATH") {
    read_file = bs_common::GetBeamSlamConfigPath() +
                "global_map/global_map_refinement.json";
  }

  if (!boost::filesystem::exists(read_file)) {
    BEAM_ERROR(
        "Cannot find global map refinement config at: {}, using default "
        "parameters.",
        read_file);
    return;
  }

  BEAM_INFO("Loading global map refinement config file: {}", read_file);

  nlohmann::json J;
  std::ifstream file(read_file);
  file >> J;
  loop_closure_candidate_search_type = J["loop_closure_candidate_search_type"];
  loop_closure_refinement_type = J["loop_closure_refinement_type"];
  loop_closure_candidate_search_config =
      J["loop_closure_candidate_search_config"];
  loop_closure_refinement_config = J["loop_closure_refinement_config"];
  scan_registration_type = J["scan_registration_type"];

  std::vector<double> vec = J["scan_reg_covariance_diag"];
  if (vec.size() != 6) {
    BEAM_ERROR(
        "Invalid local mapper covariance diagonal (6 values required). Using "
        "default.");
  } else {
    Eigen::VectorXd vec_eig(6);
    vec_eig << vec[0], vec[1], vec[2], vec[3], vec[4], vec[5];
    scan_reg_covariance = vec_eig.asDiagonal();
  }

  std::vector<double> vec2 = J["loop_closure_covariance_diag"];
  if (vec2.size() != 6) {
    BEAM_ERROR(
        "Invalid loop closure covariance diagonal (6 values required). Using "
        "default.");
  } else {
    Eigen::VectorXd vec_eig = Eigen::VectorXd(6);
    vec_eig << vec2[0], vec2[1], vec2[2], vec2[3], vec2[4], vec2[5];
    loop_closure_covariance = vec_eig.asDiagonal();
  }

  nlohmann::json J_multiscanreg = J["multi_scan_registration"];
  multi_scan_reg_params.outlier_threshold_t =
      J_multiscanreg["outlier_threshold_t"];
  multi_scan_reg_params.outlier_threshold_r =
      J_multiscanreg["outlier_threshold_r"];
  multi_scan_reg_params.min_motion_trans_m =
      J_multiscanreg["min_motion_trans_m"];
  multi_scan_reg_params.min_motion_rot_rad =
      J_multiscanreg["min_motion_rot_rad"];
  multi_scan_reg_params.max_motion_trans_m =
      J_multiscanreg["max_motion_trans_m"];
  multi_scan_reg_params.num_neighbors = J_multiscanreg["num_neighbors"];

  nlohmann::json J_scantomapreg = J["scan_to_map_registration"];
  scan_to_map_reg_params.outlier_threshold_t =
      J_scantomapreg["outlier_threshold_t"];
  scan_to_map_reg_params.outlier_threshold_r =
      J_scantomapreg["outlier_threshold_r"];
  scan_to_map_reg_params.min_motion_trans_m =
      J_scantomapreg["min_motion_trans_m"];
  scan_to_map_reg_params.min_motion_rot_rad =
      J_scantomapreg["min_motion_rot_rad"];
  scan_to_map_reg_params.max_motion_trans_m =
      J_scantomapreg["max_motion_trans_m"];
  scan_to_map_reg_params.map_size = J_scantomapreg["map_size"];

  nlohmann::json J_loammatcher = J["loam_matcher_params"];
  loam_matcher_params.max_correspondence_distance =
      J_loammatcher["max_correspondence_distance"];
  loam_matcher_params.validate_correspondences =
      J_loammatcher["validate_correspondences"];
  loam_matcher_params.iterate_correspondences =
      J_loammatcher["iterate_correspondences"];
  loam_matcher_params.convergence_criteria_translation_m =
      J_loammatcher["convergence_criteria_translation_m"];
  loam_matcher_params.convergence_criteria_rotation_deg =
      J_loammatcher["convergence_criteria_rotation_deg"];
  loam_matcher_params.max_correspondence_iterations =
      J_loammatcher["max_correspondence_iterations"];
  loam_matcher_params.output_ceres_summary =
      J_loammatcher["output_ceres_summary"];
}

GlobalMapRefinement::GlobalMapRefinement(const std::string& global_map_data_dir,
                                         const Params& params)
    : params_(params) {
  Setup();

  // load global map to get submaps
  BEAM_INFO("Loading global map data from: {}", global_map_data_dir);
  global_map_ = std::make_shared<gm::GlobalMap>(global_map_data_dir);
  submaps_ = global_map_->GetSubmaps();
}

GlobalMapRefinement::GlobalMapRefinement(const std::string& global_map_data_dir,
                                         const std::string& config_path) {
  // load params & setup
  params_.LoadJson(config_path);
  Setup();

  // load global map to get submaps
  BEAM_INFO("Loading global map data from: {}", global_map_data_dir);
  global_map_ = std::make_shared<gm::GlobalMap>(global_map_data_dir);
  submaps_ = global_map_->GetSubmaps();
}

GlobalMapRefinement::GlobalMapRefinement(
    std::shared_ptr<gm::GlobalMap>& global_map, const Params& params)
    : global_map_(global_map), params_(params) {
  submaps_ = global_map_->GetSubmaps();
  Setup();
}

GlobalMapRefinement::GlobalMapRefinement(
    std::shared_ptr<gm::GlobalMap>& global_map, const std::string& config_path)
    : global_map_(global_map) {
  params_.LoadJson(config_path);
  submaps_ = global_map_->GetSubmaps();
  Setup();
}

void GlobalMapRefinement::Setup() {
  // initiate loop closure candidate search
  if (params_.loop_closure_candidate_search_type == "EUCDIST") {
    loop_closure_candidate_search_ =
        std::make_unique<gm::LoopClosureCandidateSearchEucDist>(
            params_.loop_closure_candidate_search_config);
  } else {
    BEAM_ERROR(
        "Invalid loop closure candidate search type. Using default: EUCDIST. "
        "Input: {}",
        params_.loop_closure_candidate_search_type);
    loop_closure_candidate_search_ =
        std::make_unique<gm::LoopClosureCandidateSearchEucDist>(
            params_.loop_closure_candidate_search_config);
  }

  // initiate loop closure refinement
  if (params_.loop_closure_refinement_type == "ICP") {
    loop_closure_refinement_ = std::make_unique<gm::LoopClosureRefinementIcp>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  } else if (params_.loop_closure_refinement_type == "GICP") {
    loop_closure_refinement_ = std::make_unique<gm::LoopClosureRefinementGicp>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  } else if (params_.loop_closure_refinement_type == "NDT") {
    loop_closure_refinement_ = std::make_unique<gm::LoopClosureRefinementNdt>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  } else if (params_.loop_closure_refinement_type == "LOAM") {
    loop_closure_refinement_ = std::make_unique<gm::LoopClosureRefinementLoam>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  } else {
    BEAM_ERROR("Invalid loop closure refinement type. Using default: ICP");
    loop_closure_refinement_ = std::make_unique<gm::LoopClosureRefinementIcp>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  }
}

bool GlobalMapRefinement::RunSubmapRefinement() {
  for (uint16_t i = 0; i < submaps_.size(); i++) {
    BEAM_INFO("Refining submap No. {}", static_cast<int>(i));
    if (!RefineSubmap(submaps_.at(i))) {
      BEAM_ERROR("Submap refinement failed, exiting.");
      return false;
    }
  }
  return true;
}

bool GlobalMapRefinement::RefineSubmap(std::shared_ptr<gm::Submap>& submap) {
  // Create optimization graph
  std::shared_ptr<fuse_graphs::HashGraph> graph =
      fuse_graphs::HashGraph::make_shared();

  // create scan matching tools
  std::unique_ptr<beam_matching::LoamMatcher> matcher =
      std::make_unique<beam_matching::LoamMatcher>(params_.loam_matcher_params);

  std::unique_ptr<f2f::ScanRegistrationBase> scan_registration;

  if (params_.scan_registration_type == "MULTISCAN") {
    scan_registration = std::make_unique<f2f::MultiScanLoamRegistration>(
        std::move(matcher), params_.multi_scan_reg_params);
    scan_registration->SetFixedCovariance(params_.scan_reg_covariance);
  } else if (params_.scan_registration_type == "SCANTOMAP") {
    scan_registration = std::make_unique<f2f::ScanToMapLoamRegistration>(
        std::move(matcher), params_.scan_to_map_reg_params);
    scan_registration->SetFixedCovariance(params_.scan_reg_covariance);
  } else {
    BEAM_ERROR(
        "Invalid scan registration type. Options: MULTISCAN, SCANTOMAP. Input: "
        "{}",
        params_.scan_registration_type);
  }

  // clear lidar map
  scan_registration->GetMapMutable().Clear();

  // iterate through stored scan poses and add scan registration factors to the
  // graph
  BEAM_INFO("Registering scans");
  for (auto scan_iter = submap->LidarKeyframesBegin();
       scan_iter != submap->LidarKeyframesEnd(); scan_iter++) {
    const gm::ScanPose& scan_pose = scan_iter->second;
    auto transaction =
        scan_registration->RegisterNewScan(scan_pose).GetTransaction();
    if (transaction != nullptr) {
      graph->update(*transaction);
    }
  }

  // TODO: Add visual BA constraints

  // Optimize graph and update data
  BEAM_INFO("Optimizing graph");
  graph->optimize();

  BEAM_INFO("updating scan poses");
  for (auto scan_iter = submap->LidarKeyframesBegin();
       scan_iter != submap->LidarKeyframesEnd(); scan_iter++) {
    scan_iter->second.UpdatePose(graph);
  }

  // TODO: update visual data (just frame poses?)

  return true;
}

bool GlobalMapRefinement::RunPoseGraphOptimization() {
  // TODO
  BEAM_ERROR("PGO NOT YET IMPLEMENTED");
  return true;
}

void GlobalMapRefinement::SaveResults(const std::string& output_path,
                                      bool save_initial) {
  // create results directory
  if (!boost::filesystem::exists(output_path)) {
    BEAM_ERROR(
        "Output directory does not exist, not outputting global map refinement "
        "results. Input: {}",
        output_path);
    return;
  }

  std::string save_dir;
  if (output_path.back() != '/') {
    save_dir = output_path + "/refinement_results/";
  } else {
    save_dir = output_path + "refinement_results/";
  }
  boost::filesystem::create_directory(save_dir);

  // save
  global_map_->SaveTrajectoryFile(save_dir, save_initial);
  global_map_->SaveTrajectoryClouds(save_dir, save_initial);
  global_map_->SaveSubmapFrames(save_dir, save_initial);
  global_map_->SaveLidarSubmaps(save_dir, save_initial);
  global_map_->SaveKeypointSubmaps(save_dir, save_initial);
}

void GlobalMapRefinement::SaveGlobalMapData(const std::string& output_path) {
  // create results directory
  if (!boost::filesystem::exists(output_path)) {
    BEAM_ERROR(
        "Output directory does not exist, not outputting global map refinement "
        "results. Input: {}",
        output_path);
    return;
  }

  std::string save_dir;
  if (output_path.back() != '/') {
    save_dir = output_path + "/global_map_data_refined/";
  } else {
    save_dir = output_path + "global_map_data_refined/";
  }
  boost::filesystem::create_directory(save_dir);

  // save
  global_map_->SaveData(save_dir);
}

}  // namespace bs_tools