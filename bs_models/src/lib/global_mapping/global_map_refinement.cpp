#include <bs_models/global_mapping/global_map_refinement.h>

#include <fuse_core/transaction.h>
#include <fuse_graphs/hash_graph.h>

#include <bs_common/utils.h>
#include <bs_models/lidar/scan_pose.h>
#include <bs_models/reloc/reloc_methods.h>

namespace bs_models { namespace global_mapping {

using namespace reloc;
using namespace beam_matching;

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
      std::vector<std::string>{"loop_closure", "submap_refinement",
                               "submap_alignment"},
      J);

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
      std::vector<std::string>{"scan_registration_config", "matcher_config"},
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
  bs_common::ValidateJsonKeysOrThrow(std::vector<std::string>{"matcher_config"},
                                     J_submap_alignment);
  matcher_config_rel = J_submap_refinement["matcher_config"];
  if (!matcher_config_rel.empty()) {
    submap_alignment.matcher_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), matcher_config_rel);
  }
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
  // setup submap alignment
  const auto& m_conf = params_.submap_alignment.matcher_config;
  auto matcher_type = GetTypeFromConfig(m_conf);
  if (matcher_type == MatcherType::LOAM) {
    matcher_loam_ = std::make_unique<LoamMatcher>(LoamParams(m_conf));
  } else if (matcher_type == MatcherType::ICP) {
    matcher_ = std::make_unique<IcpMatcher>(IcpMatcher::Params(m_conf));
  } else if (matcher_type == MatcherType::GICP) {
    matcher_ = std::make_unique<GicpMatcher>(GicpMatcher::Params(m_conf));
  } else if (matcher_type == MatcherType::NDT) {
    matcher_ = std::make_unique<NdtMatcher>(NdtMatcher::Params(m_conf));
  } else {
    BEAM_ERROR("Invalid matcher type");
    throw std::invalid_argument{"invalid json"};
  }

  // set reloc params in global map which is where the reloc gets performed
  GlobalMap::Params& global_map_params = global_map_->GetParamsMutable();
  global_map_params.loop_closure_candidate_search_config =
      params_.loop_closure.candidate_search_config;
  global_map_params.loop_closure_refinement_config =
      params_.loop_closure.refinement_config;
  global_map_->Setup();
}

bool GlobalMapRefinement::RunSubmapRefinement(const std::string& output_path) {
  std::vector<SubmapPtr> submaps = global_map_->GetSubmaps();
  for (uint16_t i = 0; i < submaps.size(); i++) {
    BEAM_INFO("Refining submap No. {}", i);
    if (!RefineSubmap(submaps.at(i), output_path)) {
      BEAM_ERROR("Submap refinement failed, exiting.");
      return false;
    }
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

  for (uint16_t i = 1; i < submaps.size(); i++) {
    BEAM_INFO("Aligning submap No. {}", i);
    if (!AlignSubmaps(submaps.at(i - 1), submaps.at(i), output_path)) {
      BEAM_ERROR("Submap alignment failed, exiting.");
      return false;
    }
  }
  return true;
}

bool GlobalMapRefinement::RefineSubmap(SubmapPtr& submap,
                                       const std::string& output_path) {
  if (!output_path.empty() && !std::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path for submap refinement: {}", output_path);
    throw std::runtime_error{"invalid path"};
  }
  std::string dir = "submap_" + std::to_string(submap->Stamp().toSec());
  std::string submap_output =
      output_path.empty() ? output_path : beam::CombinePaths(output_path, dir);
  std::filesystem::create_directory(submap_output);

  // Create optimization graph
  std::shared_ptr<fuse_graphs::HashGraph> graph =
      fuse_graphs::HashGraph::make_shared();
  auto& map = bs_models::scan_registration::RegistrationMap::GetInstance();
  std::unique_ptr<sr::ScanRegistrationBase> scan_registration =
      sr::ScanRegistrationBase::Create(
          params_.submap_refinement.scan_registration_config,
          params_.submap_refinement.matcher_config, submap_output, true);

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
    if (transaction) { graph->update(*transaction); }
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

bool GlobalMapRefinement::AlignSubmaps(const SubmapPtr& submap_ref,
                                       SubmapPtr& submap_tgt,
                                       const std::string& output_path) {
  if (!output_path.empty() && !std::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path for submap alignment: {}", output_path);
    throw std::runtime_error{"invalid path"};
  }
  std::string dir = "submap_" + std::to_string(submap_tgt->Stamp().toSec());
  std::string submap_output =
      output_path.empty() ? output_path : beam::CombinePaths(output_path, dir);
  std::filesystem::create_directory(submap_output);

  Eigen::Matrix4d T_World_SubmapRef = submap_ref->T_WORLD_SUBMAP();
  Eigen::Matrix4d T_World_SubmapRef_Init = submap_ref->T_WORLD_SUBMAP_INIT();
  Eigen::Matrix4d T_World_SubmapTgt_Init = submap_tgt->T_WORLD_SUBMAP_INIT();

  // get initial relative pose
  Eigen::Matrix4d T_SubmapRef_SubmapTgt_Init =
      beam::InvertTransform(T_World_SubmapRef_Init) * T_World_SubmapTgt_Init;

  const bool use_initials = true;
  if (matcher_loam_) {
    // first get maps in their initial world frame
    LoamPointCloudPtr ref_in_ref_submap_frame =
        std::make_shared<LoamPointCloud>(
            submap_ref->GetLidarLoamPointsInWorldFrame(use_initials));
    LoamPointCloudPtr tgt_in_ref_submap_frame =
        std::make_shared<LoamPointCloud>(
            submap_tgt->GetLidarLoamPointsInWorldFrame(use_initials));

    // then transform to the reference submap frame
    Eigen::Matrix4d T_SubmapRef_WorldInit =
        beam::InvertTransform(T_World_SubmapRef_Init);
    ref_in_ref_submap_frame->TransformPointCloud(T_SubmapRef_WorldInit);
    tgt_in_ref_submap_frame->TransformPointCloud(T_SubmapRef_WorldInit);

    // align
    matcher_loam_->SetRef(ref_in_ref_submap_frame);
    matcher_loam_->SetTarget(tgt_in_ref_submap_frame);
    bool match_success = matcher_loam_->Match();
    Eigen::Matrix4d T_SubmapRef_SubmapTgt =
        matcher_loam_->ApplyResult(T_SubmapRef_SubmapTgt_Init);
    if (!output_path.empty()) {
      matcher_loam_->SaveResults(submap_output, "submap_cloud_");
    }

    // set new submap pose
    Eigen::Matrix4d T_World_SubmapTgt =
        T_World_SubmapRef * T_SubmapRef_SubmapTgt;
    submap_tgt->UpdatePose(T_World_SubmapTgt);
  } else {
    // first get maps in their initial world frame
    PointCloud ref_in_world =
        submap_ref->GetLidarPointsInWorldFrameCombined(use_initials);
    PointCloud tgt_in_world =
        submap_tgt->GetLidarPointsInWorldFrameCombined(use_initials);

    // then transform to the reference submap frame
    Eigen::Matrix4d T_SubmapRef_WorldInit =
        beam::InvertTransform(T_World_SubmapRef_Init);
    PointCloudPtr ref_in_ref_submap_frame = std::make_shared<PointCloud>();
    PointCloudPtr tgt_in_ref_submap_frame = std::make_shared<PointCloud>();
    pcl::transformPointCloud(ref_in_world, *ref_in_ref_submap_frame,
                             T_SubmapRef_WorldInit.cast<float>());
    pcl::transformPointCloud(tgt_in_world, *tgt_in_ref_submap_frame,
                             T_SubmapRef_WorldInit.cast<float>());

    // align
    matcher_->SetRef(ref_in_ref_submap_frame);
    matcher_->SetTarget(tgt_in_ref_submap_frame);
    bool match_success = matcher_->Match();
    Eigen::Matrix4d T_SubmapRef_SubmapTgt =
        matcher_->ApplyResult(T_SubmapRef_SubmapTgt_Init);
    if (!output_path.empty()) {
      matcher_->SaveResults(submap_output, "submap_cloud_");
    }

    // set new submap pose
    Eigen::Matrix4d T_World_SubmapTgt =
        T_World_SubmapRef * T_SubmapRef_SubmapTgt;
    submap_tgt->UpdatePose(T_World_SubmapTgt);
  }

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