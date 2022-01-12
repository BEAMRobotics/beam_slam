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
struct VOInitializerParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    /** subscribing topics */
    getParam<std::string>(nh, "image_topic", image_topic, "");

    /** vision configs */
    getParam<std::string>(nh, "descriptor", descriptor, "ORB");
    getParam<std::string>(nh, "detector", detector, "FAST");
    getParam<std::string>(nh, "descriptor_config", descriptor_config, "");
    getParam<std::string>(nh, "detector_config", detector_config, "");
    getParam<std::string>(nh, "tracker_config", tracker_config, "");

    // number of images to store landmark information in tracker
    getParam<int>(nh, "tracker_window_size", tracker_window_size, 100);

    // minimum amount of time between keyframes
    getParam<double>(nh, "parallax", parallax, 20.0);

  }

  std::string image_topic{};
  std::string descriptor{};
  std::string descriptor_config{};
  std::string detector{};
  std::string detector_config{};
  std::string tracker_config{};
  int tracker_window_size{};
  double parallax{20.0};
};
}} // namespace bs_parameters::models
