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
    // TODO
    // getParamRequired<std::string>(nh, "type", type);
    // getParam<float>(nh, "downsample_size", downsample_size, 0.03);
    // getParam<std::string>(nh, "scan_output_directory", scan_output_directory,
    //                       "");
    // getParam<std::string>(nh, "matcher_params_path", matcher_params_path, "");
    // getParam<std::string>(nh, "registration_config_path",
    //                       registration_config_path, "");
    // getParam<double>(nh, "matcher_noise", matcher_noise, 1e-9);

    // nh.param("matcher_noise_diagonal", matcher_noise_diagonal,
    //          matcher_noise_diagonal);

    // // get lag_duration from global namespace
    // ros::param::get("~lag_duration", lag_duration);
  }

  std::vector<double> matcher_noise_diagonal{0, 0, 0, 0, 0, 0};
  double matcher_noise;
  std::string type;
  float downsample_size;
  std::string matcher_params_path;
  std::string registration_config_path;
  std::string scan_output_directory;
  std::string imu_topic;
  std::string lidar_topic;
  ros::Duration aggregation_time{0.1};
  std::string nominal_gravity_direction{"-Z"};
  bool undistort_scans_{true};
  
};

}  // namespace models
}  // namespace beam_parameters
