#include <bs_models/global_mapping/global_map_refinement.h>

#include <fuse_core/transaction.h>
#include <fuse_graphs/hash_graph.h>

#include <bs_common/utils.h>
#include <bs_models/lidar/scan_pose.h>
#include <bs_models/reloc/reloc_methods.h>

// TODO remove
#include <bs_models/scan_registration/registration_map.h>

namespace bs_models { namespace global_mapping {

using namespace reloc;

void GlobalMapRefinement::Params::LoadJson(const std::string& config_path) {
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

  bs_common::ValidateJsonKeysOrThrow(
      std::vector<std::string>{"loop_closure", "submap_refinement"}, J);

  // load loop closure params
  nlohmann::json J_loop_closure = J["loop_closure"];
  bs_common::ValidateJsonKeysOrThrow(
      std::vector<std::string>{"candidate_search_config", "refinement_config"},
      J_loop_closure);

  std::string candidate_search_config_rel =
      J_loop_closure["candidate_search_config"];
  if (!candidate_search_config_rel.empty()) {
    loop_closure.candidate_search_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), candidate_search_config_rel);
  }

  std::string refinement_config_rel = J_loop_closure["refinement_config"];
  if (!refinement_config_rel.empty()) {
    loop_closure.refinement_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), refinement_config_rel);
  }

  // load submap refinement params
  nlohmann::json J_submap_refinement = J["submap_refinement"];
  bs_common::ValidateJsonKeysOrThrow(
      std::vector<std::string>{"scan_registration_config", "matcher_config",
                               "registration_results_output_path"},
      J_submap_refinement);

  std::string scan_registration_config_rel =
      J_submap_refinement["scan_registration_config"];
  if (!scan_registration_config_rel.empty()) {
    submap_refinement.scan_registration_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), scan_registration_config_rel);
  }

  std::string matcher_config_rel = J_submap_refinement["matcher_config"];
  if (!scan_registration_config_rel.empty()) {
    submap_refinement.matcher_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), matcher_config_rel);
  }

  submap_refinement.registration_results_output_path =
      J_submap_refinement["registration_results_output_path"];
}

GlobalMapRefinement::GlobalMapRefinement(const std::string& global_map_data_dir,
                                         const Params& params)
    : params_(params) {
  // load global map to get submaps
  BEAM_INFO("Loading global map data from: {}", global_map_data_dir);
  global_map_ = std::make_shared<GlobalMap>(global_map_data_dir);
  BEAM_INFO("Done loading global map data");
  Setup();
}

GlobalMapRefinement::GlobalMapRefinement(const std::string& global_map_data_dir,
                                         const std::string& config_path) {
  // load params & setup
  params_.LoadJson(config_path);

  // load global map to get submaps
  BEAM_INFO("Loading global map data from: {}", global_map_data_dir);
  global_map_ = std::make_shared<GlobalMap>(global_map_data_dir);
  Setup();
}

GlobalMapRefinement::GlobalMapRefinement(std::shared_ptr<GlobalMap>& global_map,
                                         const Params& params)
    : global_map_(global_map), params_(params) {
  Setup();
}

GlobalMapRefinement::GlobalMapRefinement(std::shared_ptr<GlobalMap>& global_map,
                                         const std::string& config_path)
    : global_map_(global_map) {
  params_.LoadJson(config_path);
  Setup();
}

void GlobalMapRefinement::Setup() {
  // set reloc params in global map which is where the reloc gets performed
  GlobalMap::Params& global_map_params = global_map_->GetParamsMutable();
  global_map_params.loop_closure_candidate_search_config =
      params_.loop_closure.candidate_search_config;
  global_map_params.loop_closure_refinement_config =
      params_.loop_closure.refinement_config;
  global_map_->Setup();
}

bool GlobalMapRefinement::RunSubmapRefinement() {
  std::vector<SubmapPtr> submaps = global_map_->GetSubmaps();
  for (uint16_t i = 0; i < submaps.size(); i++) {
    BEAM_INFO("Refining submap No. {}", i);
    if (!RefineSubmap(submaps.at(i))) {
      BEAM_ERROR("Submap refinement failed, exiting.");
      return false;
    }
  }
  return true;
}

bool GlobalMapRefinement::RefineSubmap(SubmapPtr& submap) {
  // Create optimization graph
  std::shared_ptr<fuse_graphs::HashGraph> graph =
      fuse_graphs::HashGraph::make_shared();
  auto& map = bs_models::scan_registration::RegistrationMap::GetInstance();
  std::unique_ptr<sr::ScanRegistrationBase> scan_registration =
      sr::ScanRegistrationBase::Create(
          params_.submap_refinement.scan_registration_config,
          params_.submap_refinement.matcher_config,
          params_.submap_refinement.registration_results_output_path, true);
  // clear lidar map
  scan_registration->GetMapMutable().Clear();

  // iterate through stored scan poses and add scan registration factors to the
  // graph
  BEAM_INFO("Registering scans");
  for (auto scan_iter = submap->LidarKeyframesBegin();
       scan_iter != submap->LidarKeyframesEnd(); scan_iter++) {
    const bs_models::ScanPose& scan_pose = scan_iter->second;
    auto transaction =
        scan_registration->RegisterNewScan(scan_pose).GetTransaction();
    if (transaction) {
      // std::cout << "\nADDING TRANSACTION: \n";
      // transaction->print();
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

bool GlobalMapRefinement::RunPoseGraphOptimization(
    const std::string& output_path) {
  global_map_->SetLoopClosureResultsPath(output_path);
  size_t num_submaps = global_map_->GetSubmaps().size();
  if (num_submaps <= pgo_skip_first_n_submaps_) {
    BEAM_ERROR("Global map size {} not large enough to run PGO, must have at "
               "least {} submaps",
               num_submaps, pgo_skip_first_n_submaps_);
  }
  BEAM_INFO("Running pose-graph optimization on submaps");
  for (int i = pgo_skip_first_n_submaps_; i < num_submaps - 1; i++) {
    global_map_->RunLoopClosure(i);
  }

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

  // save
  global_map_->SaveTrajectoryFile(output_path, save_initial);
  global_map_->SaveTrajectoryClouds(output_path, save_initial);
  global_map_->SaveSubmapFrames(output_path, save_initial);
  global_map_->SaveLidarSubmaps(output_path, save_initial);
  global_map_->SaveKeypointSubmaps(output_path, save_initial);
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

  std::string save_dir =
      beam::CombinePaths(output_path, "global_map_data_refined");
  boost::filesystem::create_directory(save_dir);

  // save
  global_map_->SaveData(save_dir);
}

}} // namespace bs_models::global_mapping