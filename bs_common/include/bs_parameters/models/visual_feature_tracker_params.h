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

    getParam<std::string>(nh, "descriptor_config", descriptor_config, "");
    getParam<std::string>(nh, "detector_config", detector_config, "");
    getParam<std::string>(nh, "tracker_config", tracker_config, "");
    getParam<std::string>(nh, "save_tracks_folder", save_tracks_folder, "");

    getParam<int>(nh, "sensor_id", sensor_id, 0);
    getParam<int>(nh, "visual_window_size", visual_window_size, 10);
  }

  // subscribing topics
  std::string image_topic{};

  // vision configs
  std::string descriptor_config{};
  std::string detector_config{};
  std::string tracker_config{};
  std::string save_tracks_folder{};

  int sensor_id{0};
  int visual_window_size{100};
};
}} // namespace bs_parameters::models
