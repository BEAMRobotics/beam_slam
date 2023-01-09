#pragma once

#include <numeric>

#include <ros/param.h>

#include <beam_utils/pointclouds.h>

#include <bs_parameters/parameter_base.h>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the LidarFeatureExtractor
 * class
 */
struct LidarFeatureExtractorParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    /** Input lidar topic (distorted) */
    getParamRequired<std::string>(nh, "pointcloud_topic", pointcloud_topic);

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

    getParam<std::string>(nh, "loam_config", loam_config_file,
                          loam_config_file);
  }

  std::string pointcloud_topic;
  LidarType lidar_type{LidarType::VELODYNE};
  std::string loam_config_file{""};
};

}} // namespace bs_parameters::models
