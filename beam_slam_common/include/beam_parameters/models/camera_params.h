#ifndef BEAM_MODELS_PARAMETERS_CAMERA_PARAMS_H
#define BEAM_MODELS_PARAMETERS_CAMERA_PARAMS_H

#include <beam_parameters/parameter_base.h>

#include <ros/node_handle.h>
#include <ros/param.h>

#include <string>
#include <vector>

namespace beam_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct CameraParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    getParam<std::string>(nh, "image_topic", image_topic, "");
    getParam<std::string>(nh, "source", source, "VIO");
    getParam<std::string>(nh, "init_path_topic", init_path_topic, "");
    getParam<int>(nh, "window_size", window_size, 100);
    getParam<std::string>(nh, "imu_topic", imu_topic, "");
  }

  std::string image_topic{};
  std::string init_path_topic{};
  int window_size{};
  std::string imu_topic{};
  std::string source{};
};

}} // namespace beam_parameters::models

#endif
