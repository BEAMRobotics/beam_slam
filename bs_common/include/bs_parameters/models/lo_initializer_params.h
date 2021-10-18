#pragma once

#include <ros/param.h>

#include <bs_parameters/parameter_base.h>

namespace bs_parameters {
namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct LoInitializerParams : public ParameterBase {
 public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    
    getParam<std::string>(nh, "registration_config_path", registration_config_path, "");
    getParam<std::string>(nh, "matcher_params_path", matcher_params_path, "");
    getParam<std::string>(nh, "ceres_config_path", ceres_config_path, "");
    getParam<std::string>(nh, "scan_output_directory", scan_output_directory,
                          "");
    getParamRequired<std::string>(nh, "lidar_topic", lidar_topic);
    getParamRequired<std::string>(nh, "output_topic", output_topic);
    getParam<double>(nh, "min_trajectory_distance", min_trajectory_distance, 3);

    getParam<int>(nh, "max_frequency", max_frequency, 5);

    double aggregation_time_double;
    getParam<double>(nh, "aggregation_time", aggregation_time_double, 0.1);
    aggregation_time = ros::Duration(aggregation_time_double);

    double trajectory_time_window_double;
    getParam<double>(nh, "trajectory_time_window", trajectory_time_window_double, 10);
    trajectory_time_window = ros::Duration(trajectory_time_window_double);
  }

  std::string registration_config_path;
  std::string matcher_params_path;
  std::string ceres_config_path;
  std::string scan_output_directory;
  std::string lidar_topic;
  std::string output_topic;
  
  double min_trajectory_distance;

  int max_frequency;

  ros::Duration aggregation_time;
  ros::Duration trajectory_time_window;
};

}  // namespace models
}  // namespace bs_parameters
