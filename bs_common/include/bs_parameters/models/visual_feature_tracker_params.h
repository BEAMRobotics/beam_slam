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
struct VisualFeatureTrackerParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    getParam<std::string>(nh, "image_topic", image_topic, "");

    getParam<std::string>(nh, "frame_initializer_config", frame_initializer_config,
                          frame_initializer_config);

    getParam<std::string>(nh, "descriptor_config", descriptor_config, "");
    getParam<std::string>(nh, "detector_config", detector_config, "");
    getParam<std::string>(nh, "tracker_config", tracker_config, "");
    getParam<std::string>(nh, "save_tracks_folder", save_tracks_folder, "");

    getParam<int>(nh, "sensor_id", sensor_id, 0);
    getParam<size_t>(nh, "visual_window_size", visual_window_size, 100);
    getParam<double>(nh, "frame_init_frequency", frame_init_frequency, 0.1);
    getParam<double>(nh, "min_trajectory_length", min_trajectory_length, 2.0);
  }

  // subscribing topics
  std::string image_topic{};

  std::string frame_initializer_config{""};

  // vision configs
  std::string descriptor_config{};
  std::string detector_config{};
  std::string tracker_config{};
  std::string save_tracks_folder{};

  int sensor_id{0};
  size_t visual_window_size{100};
  double frame_init_frequency{0.1};
  double min_trajectory_length{2.0};
};
}} // namespace bs_parameters::models
