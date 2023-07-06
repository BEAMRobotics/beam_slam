#pragma once

#include <numeric>

#include <ros/param.h>

#include <beam_utils/pointclouds.h>

#include <bs_common/utils.h>
#include <bs_parameters/parameter_base.h>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the LidarAggregation class
 */
struct LidarAggregationParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    /** This topic publishes the times that we want to motion compensate lidar
     * data to */
    getParamRequired<std::string>(nh, "aggregation_time_topic",
                                  aggregation_time_topic);

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

    std::string frame_initializer_config_rel;
    getParam<std::string>(nh, "frame_initializer_config",
                          frame_initializer_config_rel,
                          frame_initializer_config_rel);
    if (!frame_initializer_config_rel.empty()) {
      frame_initializer_config = beam::CombinePaths(
          bs_common::GetBeamSlamConfigPath(), frame_initializer_config_rel);
    }

    /** Maximum time to collect points per output scan. Default: 0.1 */
    getParam<double>(nh, "max_aggregation_time_seconds",
                     max_aggregation_time_seconds,
                     max_aggregation_time_seconds);

    /** If set to false, we will aggregate whenever the max_aggregation_time is
     * reached. Default: true*/
    getParam<bool>(nh, "use_trigger", use_trigger, use_trigger);
  }

  std::string aggregation_time_topic;
  std::string pointcloud_topic;
  LidarType lidar_type{LidarType::VELODYNE};
  double max_aggregation_time_seconds{0.1};
  bool use_trigger{true};

  std::string frame_initializer_config{""};
};

}} // namespace bs_parameters::models
