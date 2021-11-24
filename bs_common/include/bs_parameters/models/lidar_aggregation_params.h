#pragma once

#include <numeric>

#include <ros/param.h>

#include <beam_utils/pointsclouds.h>

#include <bs_parameters/parameter_base.h>

namespace bs_parameters {
namespace models {

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

    /** Output topic for aggregate, motion compensated lidar scans */
    getParamRequired<std::string>(nh, "aggregate_topic", aggregate_topic);

    /** Odometry topic to sample poses from. */
    getParamRequired<std::string>(nh, "odometry_topic", odometry_topic);

    /**
     * type of lidar. Options: VELODYNE, OUSTER. This is needed so we know how
     * to convert the PointCloud2 msgs.
     */
    lidar_type_str = "VELODYNE";
    getParam<std::string>(nh, "lidar_type", lidar_type_str, lidar_type_str);
    auto iter = beam::LidarTypeStringMap.find(lidar_type_str);
    if (iter == beam::LidarTypeStringMap.end()) {
      ROS_ERROR("Invalid lidar type input param, using default (VELODYNE). "
                "Options: %s",
                beam::GetLidarTypes().c_str());
      lidar_type = beam::LidarType::VELODYNE;
    } else {
      lidar_type = iter->second;
    }

    /** Maximum time to collect points per output scan. Default: 0.1 */
    getParam<double>(nh, "max_aggregation_time_seconds",
                     max_aggregation_time_seconds);

    /** If set to false, we will aggregate whenever the max_aggregation_time is
     * reached. Default: true*/
    getParam<bool>(nh, "use_trigger", use_trigger);

    std::string aggregation_time_topic;
    std::string odometry_topic;
    std::string pointcloud_topic;
    std::string aggregate_topic;
    beam::LidarType lidar_type{beam::LidarType::VELODYNE};
    double max_aggregation_time_seconds{0.1};
    bool use_trigger{true};
  };

} // namespace models
} // namespace bs_parameters
