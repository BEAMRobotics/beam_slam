#pragma once

#include <fuse_core/parameter.h>
#include <fuse_models/parameters/parameter_base.h>
#include <ros/param.h>

namespace bs_parameters {
namespace models {

using namespace fuse_models::parameters;

/**
 * @brief Defines the set of parameters required by the Imu3D class
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
    fuse_core::getParamRequired(nh, "init_path_topic", init_path_topic);
    getParam<bool>(nh, "init_use_scale_estimate", init_use_scale_estimate,
                   false);
    getParam<double>(nh, "keyframe_rate", keyframe_rate, 0.1);
  }

  std::string input_topic{};
  std::string init_path_topic{};
  bool init_use_scale_estimate{};
  double keyframe_rate{0.1};
};

} // namespace models
} // namespace bs_parameters
