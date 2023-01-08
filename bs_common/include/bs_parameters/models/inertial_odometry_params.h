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

    // config for an optional frame initializer
    getParam<std::string>(nh, "frame_initializer_config", frame_initializer_config,
                          frame_initializer_config);

    // if using a frame initializer, the frequency to add constraints
    getParam<double>(nh, "frame_init_frequency", frame_init_frequency, 0.1);
  }

  std::string imu_topic{""};
  std::string constraint_odom_topic{""};
  std::string frame_initializer_config{};

  double frame_init_frequency{0.1};

  int imu_hz{500};
};
}} // namespace bs_parameters::models
