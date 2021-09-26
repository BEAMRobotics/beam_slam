#pragma once

#include <ros/node_handle.h>

#include <bs_parameters/parameter_base.h>

namespace bs_parameters {
namespace models {

struct GlobalMapperParams : public ParameterBase {
 public:
  /**
   * @brief Method for loading parameter values from ROS.
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    getParam<std::string>(nh, "global_map_config", global_map_config, "");
    getParam<std::string>(nh, "output_path", output_path, "");
    getParam<std::string>(nh, "offline_map_path", offline_map_path, "");
    getParam<bool>(nh, "save_global_map_data", save_global_map_data, false);
    getParam<bool>(nh, "save_submaps", save_submaps, false);
    getParam<bool>(nh, "save_submap_frames", save_submap_frames, true);
    getParam<bool>(nh, "save_trajectory_cloud", save_trajectory_cloud, true);
    getParam<bool>(nh, "save_local_mapper_trajectory",
                   save_local_mapper_trajectory, true);
    getParam<bool>(nh, "save_local_mapper_maps", save_local_mapper_maps, false);
    getParam<bool>(nh, "publish_new_submaps", publish_new_submaps, false);
    getParam<bool>(nh, "publish_updated_global_map", publish_updated_global_map,
                   false);
    getParam<bool>(nh, "publish_new_scans", publish_new_scans, false);
  }

  std::string global_map_config;
  std::string output_path;
  std::string offline_map_path;
  bool save_global_map_data;
  bool save_submaps;
  bool save_submap_frames;
  bool save_trajectory_cloud;
  bool save_local_mapper_trajectory;
  bool save_local_mapper_maps;
  bool publish_new_submaps;
  bool publish_updated_global_map;
  bool publish_new_scans;
};

}  // namespace models
}  // namespace bs_parameters
