#pragma once

#include <bs_parameters/parameter_base.h>

#include <ros/node_handle.h>
#include <ros/param.h>

#include <string>
#include <vector>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct InertialOdometryParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    // imu topic
    getParam<std::string>(nh, "imu_topic", imu_topic, "");

    // odometry topic for the poses to add constraints to
    getParam<std::string>(nh, "constraint_odom_topic", constraint_odom_topic, "");
  }

  std::string imu_topic{""};
  std::string constraint_odom_topic{""};
};
}} // namespace bs_parameters::models
