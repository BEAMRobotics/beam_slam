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
    ros::param::get("/batch_optimizer/camera/cam_intrinsics_path",
                    cam_intrinsics_path);
    ros::param::get("/batch_optimizer/camera/image_topic", image_topic);
    ros::param::get("/batch_optimizer/camera/imu_topic", imu_topic);
    ros::param::get("/batch_optimizer/camera/window_size", window_size);
  }

  std::string cam_intrinsics_path{};
  std::string image_topic{};
  std::string imu_topic{};
  int window_size{};
};

}} // namespace beam_models::parameters

#endif
