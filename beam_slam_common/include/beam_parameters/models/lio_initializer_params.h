#pragma once

#include <ros/param.h>

#include <beam_parameters/parameter_base.h>

namespace beam_parameters {
namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct LioInitializerParams : public ParameterBase {
 public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    nh.param("matcher_noise_diagonal", matcher_noise_diagonal,
             matcher_noise_diagonal);
    getParam<double>(nh, "matcher_noise", matcher_noise, 1e-9);
    getParam<std::string>(nh, "matcher_params_path", matcher_params_path, "");
    getParam<std::string>(nh, "registration_config_path",
                          registration_config_path, "");
    getParam<std::string>(nh, "scan_output_directory", scan_output_directory,
                          "");
    getParamRequired<std::string>(nh, "imu_topic", imu_topic);
    getParamRequired<std::string>(nh, "lidar_topic", lidar_topic);
    getParamRequired<std::string>(nh, "output_topic", output_topic);
    getParam<std::string>(nh, "nominal_gravity_direction", nominal_gravity_direction, "-Z");
    getParam<bool>(nh, "undistort_scans", undistort_scans, true);
    getParam<double>(nh, "min_trajectory_distance", min_trajectory_distance, 3);
    getParam<double>(nh, "outlier_threshold_t_m", outlier_threshold_t_m, 0.3);
    getParam<double>(nh, "outlier_threshold_r_deg", outlier_threshold_r_deg, 15);

    double aggregation_time_double;
    getParam<double>(nh, "aggregation_time", aggregation_time_double, 0.1);
    aggregation_time = ros::Duration(aggregation_time_double);

    double trajectory_time_window_double;
    getParam<double>(nh, "trajectory_time_window", trajectory_time_window_double, 10);
    trajectory_time_window = ros::Duration(trajectory_time_window_double);
  }

  std::vector<double> matcher_noise_diagonal{0, 0, 0, 0, 0, 0};
  double matcher_noise;
  std::string matcher_params_path;
  std::string registration_config_path;
  std::string scan_output_directory;
  std::string imu_topic;
  std::string lidar_topic;
  std::string output_topic;
  
  std::string nominal_gravity_direction;
  bool undistort_scans{true};
  double min_trajectory_distance;
  double outlier_threshold_t_m;
  double outlier_threshold_r_deg;

  ros::Duration aggregation_time;
  ros::Duration trajectory_time_window;
};

}  // namespace models
}  // namespace beam_parameters
