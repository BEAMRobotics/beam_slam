#pragma once

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
    getParam<std::string>(nh, "imu_topic", imu_topic, "");
    getParam<std::string>(nh, "output_topic", output_topic, "");
    getParam<std::string>(nh, "pose_file_path", pose_file_path, "");
    getParam<double>(nh, "min_trajectory_length", min_trajectory_length, 0.5);

    double trajectory_time_window_double;
    getParam<double>(nh, "trajectory_time_window",
                     trajectory_time_window_double, 10);
    trajectory_time_window = ros::Duration(trajectory_time_window_double);
  }

  std::string pose_file_path;
  std::string imu_topic;
  std::string output_topic;
  double min_trajectory_length;

  ros::Duration trajectory_time_window;
};

}} // namespace bs_parameters::models
