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
 -refinement_config ~/beam_slam/beam_slam_launch/config/global_map/global_map_refinement_test.json \ 
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
DEFINE_bool(run_submap_refinement, true,
            "Set to true to refine the submaps before running the pose graph "
            "optimization. This should always be set to true, but there are "
            "possible reasons for skipping this step.");
DEFINE_bool(run_posegraph_optimization, true,
            "Set to true to run pose graph optimization after submap "
            "refinement to refine the relative pose of the submaps. "
            "This should always be set to true, but there are "
            "possible reasons for skipping this step.");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // setup ros and load calibration
  std::string name = "global_map_refinement";
  int arg = 0;
  ros::init(arg, NULL, name);
  std::string calibration_load_cmd = "rosparam load " + FLAGS_calibration_yaml;
  BEAM_INFO("Running command: {}", calibration_load_cmd);
  int result1 = system(calibration_load_cmd.c_str());

  bs_models::global_mapping::GlobalMapRefinement refinement(
      FLAGS_globalmap_dir, FLAGS_refinement_config);

  std::string save_path =
      beam::CombinePaths(FLAGS_output_path, "global_map_refined_results");
  std::filesystem::remove_all(save_path);
  std::filesystem::create_directory(save_path);
  std::string global_map_data_path =
      beam::CombinePaths(save_path, "GlobalMapData");
  std::filesystem::create_directory(global_map_data_path);

  if (FLAGS_run_submap_refinement) {
    if (!refinement.RunSubmapRefinement()) {
      BEAM_ERROR("Submap refinement failed, exiting global map refinement.");
      return 0;
    }
  } else {
    BEAM_INFO("Skipping submap refinement.");
  }

  if (FLAGS_run_posegraph_optimization) {
    std::string lc_save_path =
        beam::CombinePaths(save_path, "loop_closure_results");
    std::filesystem::create_directory(lc_save_path);
    if (!refinement.RunPoseGraphOptimization(lc_save_path)) {
      BEAM_ERROR(
          "Pose graph optimization failed, exiting global map refinement.");
      return 0;
    }
  } else {
    BEAM_INFO("Skipping pose graph optimization");
  }

  BEAM_INFO("Global map refinement completed successfully.");

  BEAM_INFO("Outputting results to: {}", save_path);
  refinement.SaveResults(save_path, true);

  BEAM_INFO("Outputting global map data to: {}", global_map_data_path);
  refinement.SaveGlobalMapData(global_map_data_path);

  return 0;
}
