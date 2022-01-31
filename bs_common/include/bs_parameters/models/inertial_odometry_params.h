#pragma once

#include <fuse_core/parameter.h>
#include <ros/param.h>

namespace bs_parameters {
namespace models {

/**
 * @brief Defines the set of parameters required by the InertialOdometry class
 */
struct InertialOdometryParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle &nh) final {
    // Required
    fuse_core::getParamRequired(nh, "input_topic", input_topic);
    getParam<bool>(nh, "init_use_scale_estimate", init_use_scale_estimate,
                   false);
    getParam<double>(nh, "keyframe_rate", keyframe_rate, 0.1);
    getParam<int>(nh, "msg_queue_size", msg_queue_size, 10000);
  }

  int msg_queue_size{};
  std::string input_topic{};
  bool init_use_scale_estimate{};
  double keyframe_rate{0.1};
};

} // namespace models
} // namespace bs_parameters
