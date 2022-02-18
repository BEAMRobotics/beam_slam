#pragma once

#include <Eigen/Dense>
#include <ros/param.h>

#include <bs_parameters/parameter_base.h>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct GTInitializerParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    // imu topic
    getParam<std::string>(nh, "imu_topic", imu_topic, "");

    /** Options: TRANSFORM, ODOMETRY, POSEFILE */
    getParam<std::string>(nh, "frame_initializer_config",
                          frame_initializer_config, frame_initializer_config);

    // minimum trajectory length for a valid initialization
    getParam<double>(nh, "min_trajectory_length", min_trajectory_length, 0.5);

    // maximum time to wait for a valid init trajectory
    double trajectory_time_window_double;
    getParam<double>(nh, "trajectory_time_window",
                     trajectory_time_window_double, 10);
    trajectory_time_window = ros::Duration(trajectory_time_window_double);
  }

  std::string frame_initializer_config{""};
  std::string imu_topic;
  double min_trajectory_length;
  ros::Duration trajectory_time_window;
};

}} // namespace bs_parameters::models
