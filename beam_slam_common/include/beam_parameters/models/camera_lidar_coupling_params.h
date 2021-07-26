#pragma once

#include <beam_parameters/parameter_base.h>

namespace beam_parameters {
namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct CameraLidarCouplingParams : public ParameterBase {
 public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    getParamRequired<std::string>(nh, "input_topic", input_topic);
    // getParam<int>(nh, "keypoint_queue_size", keypoint_queue_size,
    //                       keypoint_queue_size);
  }

  std::string input_topic;
  
};

}  // namespace models
}  // namespace beam_parameters
