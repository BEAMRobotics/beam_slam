#pragma once

#include <numeric>

#include <ros/param.h>

#include <beam_utils/pointclouds.h>

#include <bs_parameters/parameter_base.h>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the LidarAggregation class
 */
struct LidarScanDeskewerParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    /** Input lidar topic (distorted) */
    getParamRequired<std::string>(nh, "input_topic", input_topic);

    /** While waiting for IMU data, we will store a scan queue with this buffer
     * size */
    getParam<int>(nh, "scan_buffer_size", scan_buffer_size, scan_buffer_size);

    /**
     * type of lidar. Options: VELODYNE, OUSTER. This is needed so we know how
     * to convert the PointCloud2 msgs.
     */
    std::string lidar_type_str = "VELODYNE";
    getParam<std::string>(nh, "lidar_type", lidar_type_str, lidar_type_str);
    auto iter = LidarTypeStringMap.find(lidar_type_str);
    if (iter == LidarTypeStringMap.end()) {
      ROS_ERROR("Invalid lidar type input param, using default (VELODYNE). "
                "Options: %s",
                GetLidarTypes().c_str());
      lidar_type = LidarType::VELODYNE;
    } else {
      lidar_type = iter->second;
    }

    getParam<std::string>(nh, "frame_initializer_config",
                          frame_initializer_config, frame_initializer_config);
  }

  int scan_buffer_size{5};
  std::string input_topic;
  LidarType lidar_type{LidarType::VELODYNE};
  std::string frame_initializer_config{""};
};

}} // namespace bs_parameters::models
