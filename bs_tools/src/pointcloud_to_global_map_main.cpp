#include <gflags/gflags.h>

#include <beam_utils/gflags.h>
#include <bs_tools/pointcloud_to_global_map.h>

DEFINE_string(pointcloud, "",
              "Full path to PLY or PCD file to convert to global map.");
DEFINE_validator(pointcloud, &beam::gflags::ValidateFileMustExist);
DEFINE_string(
    config_path, "",
    "Full path to config file for the map conversion. If left empty, this will "
    "use the default parameters defined in the class header. If set to "
    "DEFAULT_PATH, it will lookup the config file in "
    "beam_slam/beam_slam_launch/config/pointcloud_to_global_map.json");
DEFINE_string(output_path, "", "Full path to output directory.");
DEFINE_validator(output_path, &beam::gflags::ValidateDirMustExist);
DEFINE_bool(output_data, true,
            "Set to true to output all global map data so that it can be "
            "re-loaded later.");
DEFINE_bool(output_results, true,
            "Set to true to output all results in an easily viewable form "
            "including lidar maps, keypoint maps, and trajectories.");

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  bs_tools::PointcloudToGlobalMap converter(FLAGS_pointcloud,
                                            FLAGS_config_path);

  std::string output_path = FLAGS_output_path;

  if (output_path.back() != '/') {
    output_path += "/";
  }

  if (FLAGS_output_data) {
    std::string data_path = output_path + "GlobalMapData/";
    boost::filesystem::create_directory(data_path);
    BEAM_INFO("Saving global map data to {}", data_path);
    if (!converter.SaveGlobalMapData(data_path)) {
      BEAM_ERROR("Failed to save global map data, check output path");
    }
  }

  if (FLAGS_output_results) {
    std::string results_path = output_path + "GlobalMapResults/";
    boost::filesystem::create_directory(results_path);
    BEAM_INFO("Saving global map results to {}", results_path);
    if (!converter.SaveGlobalMapResults(results_path)) {
      BEAM_ERROR("Failed to save global map results, check output path");
    }
  }

  return 0;
}
