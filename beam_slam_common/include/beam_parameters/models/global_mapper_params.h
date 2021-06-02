#pragma once

#include <ros/node_handle.h>

#include <beam_common/sensor_config.h>

#include <beam_parameters/parameter_base.h>

namespace beam_parameters { namespace models {

struct GlobalMapperParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
     getParam<std::string>(nh, "global_mapper_config", global_mapper_config, "");
     getParam<std::string>(nh, "output_path", output_path, "");
     getParam<bool>(nh, "save_submaps", save_submaps, false);
     getParam<bool>(nh, "save_final_map", save_final_map, false);
     getParam<bool>(nh, "save_trajectory_cloud", save_trajectory_cloud, true);
     getParam<bool>(nh, "save_local_mapper_trajectory", save_local_mapper_trajectory, true);
     getParam<bool>(nh, "save_local_mapper_maps", save_local_mapper_maps, false);

    // get extrinsics params from global namespace
    ros::param::get("~imu_frame", imu_frame);
    ros::param::get("~lidar_frame", lidar_frame);
    ros::param::get("~camera_frame", camera_frame);
    ros::param::get("~static_extrinsics", static_extrinsics);
    ros::param::get("~intrinsics_path", intrinsics_path);
  }

  std::string global_mapper_config;
  std::string output_path;
  std::string imu_frame;
  std::string lidar_frame;
  std::string camera_frame;
  std::string intrinsics_path;
  bool save_submaps;
  bool save_final_map;
  bool save_trajectory_cloud;
  bool static_extrinsics{true};
  bool save_local_mapper_trajectory{true};
  bool save_local_mapper_maps{false};

};

}} // namespace beam_parameters::models
