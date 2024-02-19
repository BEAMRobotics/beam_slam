#include <filesystem>

#include <gflags/gflags.h>

#include <beam_utils/gflags.h>
#include <bs_models/global_mapping/global_map_refinement.h>

// clang-format off
/** 
 * Example command for running binary:
 * 
 ./devel/lib/bs_tools/bs_tools_global_map_refinement_main \
 -globalmap_dir ~/results/global_mapper/global_mapper_results/GlobalMapData/ \
 -output_path ~/results \
 -run_submap_refinement=true \
 -run_posegraph_optimization=true \ 
 -refinement_config ~/beam_slam/beam_slam_launch/config/global_map/global_map_refinement.json \ 
 -calibration_yaml ~/beam_slam/beam_slam_launch/config/calibration_params.yaml
*
* NOTE: YOU MUST ALSO PUBLISH YOUR EXTRINSIC CALIBRATIONS - USE: calibration_publisher.launch
*/
// clang-format on

DEFINE_string(globalmap_dir, "",
              "Full path to global map directory to load (Required).");
DEFINE_validator(globalmap_dir, &beam::gflags::ValidateDirMustExist);
DEFINE_string(
    refinement_config, "",
    "Full path to config file for the map refinement. If left empty, this will "
    "use the default parameters defined in the class header. You can use the "
    "default in: "
    ".../beam_slam/beam_slam_launch/config/global_map/"
    "global_map_refinement.json");
DEFINE_string(output_path, "", "Full path to output directory. ");
DEFINE_validator(output_path, &beam::gflags::ValidateDirMustExist);
DEFINE_string(calibration_yaml, "", "Full path to calibration yaml. ");
DEFINE_validator(calibration_yaml, &beam::gflags::ValidateFileMustExist);
DEFINE_bool(run_batch_optimizer, true,
            "Set to true to run registration in batch with loop closures along "
            "the whole trajectory.");
DEFINE_bool(run_submap_refinement, true,
            "Set to true to refine the submaps before running the pose graph "
            "optimization.");
DEFINE_bool(run_submap_alignment, true,
            "Set to true to re-align the submaps after running refinement and "
            "before running the pose graph optimization. ");
DEFINE_bool(run_posegraph_optimization, true,
            "Set to true to run pose graph optimization after submap "
            "refinement to refine the relative pose of the submaps. ");

// Global Variables
bool RUN_BATCH = false;
bool RUN_REFINEMENT = false;
bool RUN_ALIGNMENT = false;
bool RUN_PGO = false;
std::string SAVE_PATH;

int RunBatchOptimizer(
    bs_models::global_mapping::GlobalMapRefinement& refinement,
    bs_models::global_mapping::GlobalMap& map) {
  if (!RUN_BATCH) {
    BEAM_INFO("Skipping batch optimization.");
    return 0;
  }

  std::string batch_save_path =
      beam::CombinePaths(SAVE_PATH, "batch_optimization_results");
  std::filesystem::create_directory(batch_save_path);

  if (!refinement.RunBatchOptimization(batch_save_path)) {
    BEAM_ERROR("Batch optimization failed");
    return 1;
  }
  return 0;
}

int RunRefinement(bs_models::global_mapping::GlobalMapRefinement& refinement,
                  bs_models::global_mapping::GlobalMap& map) {
  if (!RUN_REFINEMENT) {
    BEAM_INFO("Skipping submap refinement.");
    return 0;
  }

  std::string refinement_save_path =
      beam::CombinePaths(SAVE_PATH, "submap_refinement_results");
  std::filesystem::create_directory(refinement_save_path);

  if (!refinement.RunSubmapRefinement(refinement_save_path)) {
    BEAM_ERROR("Submap refinement failed, exiting global map refinement.");
    return 1;
  }
  return 0;
}

int RunAlignment(bs_models::global_mapping::GlobalMapRefinement& refinement,
                 bs_models::global_mapping::GlobalMap& map) {
  if (!RUN_ALIGNMENT) {
    BEAM_INFO("Skipping submap alignment.");
    return 0;
  }
  std::string alignment_save_path =
      beam::CombinePaths(SAVE_PATH, "submap_alignment_results");
  std::filesystem::create_directory(alignment_save_path);

  if (!refinement.RunSubmapAlignment(alignment_save_path)) {
    BEAM_ERROR("Submap alignment failed, exiting global map refinement.");
    return 1;
  }
  return 0;
}

int RunPGO(bs_models::global_mapping::GlobalMapRefinement& refinement,
           bs_models::global_mapping::GlobalMap& map) {
  if (!RUN_PGO) {
    BEAM_INFO("Skipping pose graph optimization");
    return 0;
  }

  std::string lc_save_path =
      beam::CombinePaths(SAVE_PATH, "loop_closure_results");
  std::filesystem::create_directory(lc_save_path);

  if (!refinement.RunPoseGraphOptimization(lc_save_path)) {
    BEAM_ERROR(
        "Pose graph optimization failed, exiting global map refinement.");
    return 1;
  }
  return 0;
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // setup ros and load calibration
  int arg = 0;
  ros::init(arg, NULL, "global_map_refinement");
  std::string calibration_load_cmd = "rosparam load " + FLAGS_calibration_yaml;
  BEAM_INFO("Running command: {}", calibration_load_cmd);
  int result1 = system(calibration_load_cmd.c_str());

  // set global variables
  RUN_BATCH = FLAGS_run_batch_optimizer;
  RUN_REFINEMENT = FLAGS_run_submap_refinement;
  RUN_ALIGNMENT = FLAGS_run_submap_alignment;
  RUN_PGO = FLAGS_run_posegraph_optimization;
  SAVE_PATH =
      beam::CombinePaths(FLAGS_output_path, "global_map_refined_results");

  // setup output
  if (std::filesystem::exists(SAVE_PATH)) {
    BEAM_INFO("Clearing output path: {}", SAVE_PATH);
    std::filesystem::remove_all(SAVE_PATH);
  }
  std::filesystem::create_directory(SAVE_PATH);

  // load global map and refinement
  BEAM_INFO("Loading global map data from: {}", FLAGS_globalmap_dir);
  auto global_map = std::make_shared<bs_models::global_mapping::GlobalMap>(
      FLAGS_globalmap_dir);
  bs_models::global_mapping::GlobalMapRefinement refinement(
      global_map, FLAGS_refinement_config);

  // run refinement
  if (RunBatchOptimizer(refinement, *global_map) != 0) { return 1; }
  if (RunRefinement(refinement, *global_map) != 0) { return 1; }
  if (RunAlignment(refinement, *global_map) != 0) { return 1; }
  if (RunPGO(refinement, *global_map) != 0) { return 1; }
  BEAM_INFO("Global map refinement completed successfully.");

  // output results
  BEAM_INFO("Outputting results to: {}", SAVE_PATH);
  refinement.SaveResults(SAVE_PATH, true);

  // Save global map data
  std::string global_map_data_path =
      beam::CombinePaths(SAVE_PATH, "GlobalMapData");
  std::filesystem::create_directory(global_map_data_path);
  BEAM_INFO("Outputting global map data to: {}", global_map_data_path);
  refinement.SaveGlobalMapData(global_map_data_path);

  return 0;
}
