#include <gflags/gflags.h>

#include <beam_utils/gflags.h>
#include <bs_models/global_mapping/global_map_refinement.h>
#include <bs_models/global_mapping/global_map.h>
#include <bs_models/global_mapping/submap.h>

DEFINE_string(globalmap_dir, "",
              "Full path to global map directory to load (Required).");
DEFINE_validator(globalmap_dir, &beam::gflags::ValidateDirMustExist);
DEFINE_string(
    refinement_config, "",
    "Full path to config file for the map refinement. If left empty, this will "
    "use the default parameters defined in the class header. If set to "
    "DEFAULT_PATH, it will lookup the config file in "
    ".../beam_slam/beam_slam_launch/config/global_map/"
    "global_map_refinement.json");
DEFINE_string(output_path, "", "Full path to output directory.");
DEFINE_validator(output_path, &beam::gflags::ValidateDirMustExist);
DEFINE_bool(output_globalmap_data, true,
            "Set to true to output all global map data so that it can be "
            "re-loaded later.");
DEFINE_bool(output_results, true,
            "Set to true to output all results in an easily viewable form "
            "including lidar maps, keypoint maps, and trajectories.");
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

  bs_models::global_mapping::GlobalMap global_map;
  if (!global_map.Load(FLAGS_globalmap_dir)) {
    BEAM_ERROR("Cannot load global map, exiting global map refinement.");
    return 0;
  }

  std::vector<bs_models::global_mapping::Submap>& submaps =
      global_map.GetSubmaps();

  bs_models::global_mapping::GlobalMapRefinement refinement(
      submaps, FLAGS_refinement_config);

  if (FLAGS_run_submap_refinement) {
    if (!refinement.RunSubmapRefinement()) {
      BEAM_ERROR("Submap refinement failed, exiting global map refinement.");
      return 0;
    }
  }

  if (FLAGS_run_posegraph_optimization) {
    if (!refinement.RunPoseGraphOptimization()) {
      BEAM_ERROR(
          "Pose graph optimization failed, exiting global map refinement.");
      return 0;
    }
  }

  BEAM_INFO("Global map refinement completed successfully.");

  std::string dateandtime =
      beam::ConvertTimeToDate(std::chrono::system_clock::now());

  if (FLAGS_output_path.back() != '/') {
    dateandtime = "/" + dateandtime;
  }

  if (FLAGS_output_results) {
    std::string save_path =
        FLAGS_output_path + dateandtime + "_global_map_refined_results/";
    boost::filesystem::create_directory(save_path);
    BEAM_INFO("Outputting results to: {}", save_path);

    global_map.SaveTrajectoryFile(save_path, true);
    global_map.SaveTrajectoryClouds(save_path, true);
    global_map.SaveSubmapFrames(save_path, true);
    global_map.SaveLidarSubmaps(save_path, true);
    global_map.SaveKeypointSubmaps(save_path, true);
  }

  if (FLAGS_output_globalmap_data) {
    std::string save_path =
        FLAGS_output_path + dateandtime + "_global_map_refined_data/";
    boost::filesystem::create_directory(save_path);

    BEAM_INFO("Outputting global map data to: {}", save_path);
    global_map.SaveData(save_path);
  }
  return 0;
}
