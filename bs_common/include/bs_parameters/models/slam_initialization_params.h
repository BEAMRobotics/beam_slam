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
struct SLAMInitializationParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    // subscribing topics
    getParam<std::string>(nh, "visual_measurement_topic", visual_measurement_topic,
                          "/local_mapper/visual_feature_tracker/visual_measurements");
    getParam<std::string>(nh, "imu_topic", imu_topic, "");
    getParam<std::string>(nh, "lidar_topic", lidar_topic, "");

    // where tou save the initialization result
    getParam<std::string>(nh, "output_directory", output_directory, "");

    // config for an optional frame initializer
    getParam<std::string>(nh, "frame_initializer_config", frame_initializer_config,
                          frame_initializer_config);

    // mode for initializing, options: VISUAL, LIDAR, FRAMEINIT
    getParam<std::string>(nh, "init_mode", init_mode, init_mode);
    if (init_mode != "VISUAL" && init_mode != "LIDAR" && init_mode != "FRAMEINIT") {
      ROS_ERROR("Invalid init mode type, options: 'VISUAL', 'LIDAR', 'FRAMEINIT'.");
    }
    // maximum optimizaiton time in seconds
    getParam<double>(nh, "max_optimization_s", max_optimization_s, 1.0);

    // minimum parallax to intialize (if using VISUAL)
    getParam<double>(nh, "min_parallax", min_parallax, 20.0);

    // minimum acceptable trajectory length to intialize (if using FRAMEINIT or LIDAR)
    getParam<double>(nh, "min_trajectory_length_m", min_trajectory_length_m, 2.0);

    // size of init window in seconds
    getParam<double>(nh, "initialization_window_s", initialization_window_s, 5.0);

    // sensor frequencies
    getParam<int>(nh, "imu_hz", imu_hz, 500);
    getParam<int>(nh, "lidar_hz", lidar_hz, 10);
    getParam<int>(nh, "camera_hz", camera_hz, 20);
  }

  std::string visual_measurement_topic{"/local_mapper/visual_feature_tracker/visual_measurements"};
  std::string imu_topic{};
  std::string lidar_topic{};
  std::string frame_initializer_config{};
  std::string init_mode{"FRAMEINIT"};

  std::string output_directory{};
  double max_optimization_s{1.0};
  double min_parallax{20.0};
  double min_trajectory_length_m{2.0};
  double frame_init_frequency{0.1};

  double initialization_window_s{5.0};
  int imu_hz{500};
  int lidar_hz{10};
  int camera_hz{20};
};
}} // namespace bs_parameters::models