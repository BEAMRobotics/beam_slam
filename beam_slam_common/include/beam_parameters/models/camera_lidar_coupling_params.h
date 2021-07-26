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
    getParam<double>(nh, "correspondence_distance_theshold", correspondence_distance_theshold,
                          correspondence_distance_theshold);
  }

  std::string input_topic;
  double correspondence_distance_theshold{0.3};
  
};

}  // namespace models
}  // namespace beam_parameters
