// #include <gflags/gflags.h>
#include <beam_calibration/CameraModel.h>
// #include <beam_containers/LandmarkContainer.h>
// #include <beam_containers/LandmarkMeasurement.h>

// // #include <beam_utils/gflags.h>
// #include <bs_models/global_mapping/global_map_refinement.h>
// #include <bs_models/global_mapping/global_map.h>
// #include <bs_models/global_mapping/submap.h>

// DEFINE_string(globalmap_dir, "",
//               "Full path to global map directory to load (Required).");
// // DEFINE_validator(globalmap_dir, &beam::gflags::ValidateDirMustExist);
// DEFINE_string(
//     refinement_config, "",
//     "Full path to config file for the map refinement. If left empty, this will "
//     "use the default parameters defined in the class header. If set to "
//     "DEFAULT_PATH, it will lookup the config file in "
//     ".../beam_slam/beam_slam_launch/config/global_map/"
//     "global_map_refinement.json");
// DEFINE_bool(run_submap_refinement, true,
//             "Set to true to refine the submaps before running the pose graph "
//             "optimization. This should always be set to true, but there are "
//             "possible reasons for skipping this step.");
// DEFINE_bool(run_posegraph_optimization, true,
//             "Set to true to run pose graph optimization after submap "
//             "refinement to refine the relative pose of the submaps. "
//             "This should always be set to true, but there are "
//             "possible reasons for skipping this step.");

int main(int argc, char* argv[]) {
//   gflags::ParseCommandLineFlags(&argc, &argv, true);
//
//   bs_models::global_mapping::GlobalMap global_map;
//   if (!global_map.Load(FLAGS_globalmap_dir)) {
//     // BEAM_ERROR("Cannot load global map, exiting global map refinement.");
//     return 0;
//   }
//
//   std::vector<bs_models::global_mapping::Submap>& submaps =
//       global_map.GetSubmaps();
//
//   bs_models::global_mapping::GlobalMapRefinement refinement(
//       submaps, FLAGS_refinement_config);
//
//   if (FLAGS_run_submap_refinement) {
//     if (!refinement.RunSubmapRefinement()) {
//     //   BEAM_ERROR("Submap refinement failed, exiting global map refinement.");
//       return 0;
//     }
//   }
//
//   if (FLAGS_run_posegraph_optimization) {
//     if (!refinement.RunPoseGraphOptimization()) {
//     //   BEAM_ERROR(
//     //       "Pose graph optimization failed, exiting global map refinement.");
//       return 0;
//     }
//   }
//
// //   BEAM_INFO("Global map refinement completed successfully.");
  std::string test_str = "test";
  std::shared_ptr<beam_calibration::CameraModel> model = beam_calibration::CameraModel::Create(test_str);

//   beam_containers::LandmarkContainer<beam_containers::LandmarkMeasurement> ls;

  return 0;
}
