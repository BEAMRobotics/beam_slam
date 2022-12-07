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
    /** subscribing topics */
    getParam<std::string>(
        nh, "visual_measurement_topic", visual_measurement_topic,
        "/local_mapper/visual_feature_tracker/visual_measurements");
    getParam<std::string>(nh, "imu_topic", imu_topic, "");
    getParam<std::string>(nh, "lidar_topic", lidar_topic, "");

    // where tou save the initialization result
    getParam<std::string>(nh, "output_directory", output_directory, "");

    // config for an optional frame initializer
    getParam<std::string>(nh, "frame_initializer_config",
                          frame_initializer_config, frame_initializer_config);

    // number of images to store landmark information in container before
    // quelling
    getParam<int>(nh, "max_container_size", max_container_size, 100);

    // whether to scale the initial estimate
    getParam<bool>(nh, "use_scale_estimate", use_scale_estimate, true);

    // minimum visual parallax to attempt initializaing
    getParam<double>(nh, "min_parallax", min_parallax, 20.0);

    // maximum optimizaiton time in seconds
    getParam<double>(nh, "max_optimization_s", max_optimization_s, 1.0);
  }

  std::string visual_measurement_topic{
      "/local_mapper/visual_feature_tracker/visual_measurements"};
  std::string imu_topic{};
  std::string lidar_topic{};
  std::string frame_initializer_config{""};

  std::string output_directory{};
  double max_optimization_s{1.0};
  bool use_scale_estimate{true};
  int max_container_size{100};
  double min_parallax{20.0};
};
}} // namespace bs_parameters::models
