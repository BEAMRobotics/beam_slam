#pragma once

#include <string>
#include <vector>

#include <ros/node_handle.h>
#include <ros/param.h>

#include <bs_common/utils.h>
#include <bs_parameters/parameter_base.h>

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
    std::string descriptor_config_rel;
    std::string detector_config_rel;
    std::string tracker_config_rel;

    getParamRequired<std::string>(nh, "image_topic", image_topic);
    getParamRequired<std::string>(nh, "descriptor_config",
                                  descriptor_config_rel);
    getParamRequired<std::string>(nh, "detector_config", detector_config_rel);
    getParamRequired<std::string>(nh, "tracker_config", tracker_config_rel);

    descriptor_config = beam::CombinePaths(bs_common::GetBeamSlamConfigPath(),
                                           descriptor_config_rel);
    detector_config = beam::CombinePaths(bs_common::GetBeamSlamConfigPath(),
                                         detector_config_rel);
    tracker_config = beam::CombinePaths(bs_common::GetBeamSlamConfigPath(),
                                        tracker_config_rel);

    getParam<int>(nh, "sensor_id", sensor_id, 0);
    getParam<int>(nh, "visual_window_size", visual_window_size, 10);
  }

  // subscribing topics
  std::string image_topic{};

  // vision configs
  std::string descriptor_config{};
  std::string detector_config{};
  std::string tracker_config{};

  int sensor_id{0};
  int visual_window_size{100};
};
}} // namespace bs_parameters::models
