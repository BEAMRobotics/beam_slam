#include <bs_tools/pointcloud_to_global_map.h>

#include <beam_utils/log.h>

#include <bs_common/utils.h>

namespace bs_tools {

PointcloudToGlobalMap::Params::Params() { test = true }

void PointcloudToGlobalMap::Params::LoadJson(const std::string &config_path) {
  std::string read_file = config_path;
  if (read_file.empty()) {
    BEAM_INFO("No config file provided to pointcloud to global map converter, "
              "using default parameters.");
    return;
  }

  if (read_file == "DEFAULT_PATH") {
    read_file =
        bs_common::GetBeamSlamConfigPath() + "global_map/global_map.json";
  }

  BEAM_INFO("Loading global map config file: {}", read_file);

  nlohmann::json J;
  if (!beam::ReadJson(read_file, J)) {
    BEAM_ERROR("Using default parameters.");
    return;
  }

  submap_distance = J["submap_distance_m"];
  submap_size = J["submap_size_m"];

  if (submap_distance > submap_size) {
    BEAM_WARN("Submap size less than submap distance specified.  Submaps will "
              "not overlap, and some pointcloud regions will be lost.")
  }

  nominal_gravity_direction = J["nominal_gravity_direction"];
}

PointcloudToGlobalMap::PointcloudToGlobalMap(const std::string &pointcloud,
                                             const Params &params = Params())
    : pointcloud_(pointcloud), params_(params) {
  CreateGlobalMap();
}

PointcloudToGlobalMap::PointcloudToGlobalMap(const std::string &pointcloud,
                                             const std::string &config_path)
    : pointcloud_(pointcloud) {
  params_.LoadJson(config_path);
  CreateGlobalMap();
}

bool PointcloudToGlobalMap::CreateGlobalMap() {
  // construct global map
  global_map_ = std::make_shared<GlobalMap>(nullptr, nullptr);

  // read point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloud_, *cloud) ==
      -1) //* load the file
  {
    BEAM_ERROR("Couldn't read valid PCD file from location {}", pointcloud_);
    return false;
  }

  // compute submap center points

  // crop each submap and add to global map
}

bool PointcloudToGlobalMap::SaveGlobalMapData(std::string &output_path) {
  // verify output_path
  if (!boost::filesystem::exists(output_path)) {
    BEAM_INFO("Output directory does not exist, can not save global map "
              "data. Input: {}",
              output_path);
    return false;
  }

  // save
  global_map_->SaveData(output_path);
  return true;
}

bool PointcloudToGlobalMap::SaveGlobalMapResults(std::string &output_path) {
  // verify output_path
  if (!boost::filesystem::exists(output_path)) {
    BEAM_INFO("Output directory does not exist, can not save global map "
              "results. Input: {}",
              output_path);
    return false;
  }

  // save
  global_map_->SaveTrajectoryFile(output_path);
  global_map_->SaveTrajectoryClouds(output_path);
  global_map_->SaveSubmapFrames(output_path);
  global_map_->SaveLidarSubmaps(output_path);
  global_map_->SaveKeypointSubmaps(output_path);
  return true;
}

} // namespace bs_tools