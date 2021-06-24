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
struct VIOParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    ros::param::get("/batch_optimizer/VIO/cam_intrinsics_path",
                    cam_intrinsics_path);
    ros::param::get("/batch_optimizer/VIO/image_topic", image_topic);
    ros::param::get("/batch_optimizer/VIO/imu_topic", imu_topic);
    ros::param::get("/batch_optimizer/VIO/window_size", window_size);
    ros::param::get("/batch_optimizer/VIO/imu_intrinsics", imu_intrinsics);
    ros::param::get("/batch_optimizer/VIO/lidar_init_path_topic",
                    lidar_init_path_topic);
  }

  std::string cam_intrinsics_path{};
  std::string image_topic{};
  std::string imu_topic{};
  std::string lidar_init_path_topic{};
  int window_size{};
  std::vector<double> imu_intrinsics{0, 0, 0, 0};
};

}} // namespace beam_parameters::models

#endif
