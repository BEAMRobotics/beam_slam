#ifndef bs_models_PARAMETERS_GLOBAL_PARAMS_H
#define bs_models_PARAMETERS_GLOBAL_PARAMS_H

#include <bs_parameters/parameter_base.h>

#include <ros/node_handle.h>
#include <ros/param.h>

#include <string>
#include <vector>

namespace bs_parameters {
namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct CalibrationParams : public ParameterBase {
 public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh = ros::NodeHandle()) final {
    ros::param::get("/calibration_params/imu_intrinsics_path",
                    imu_intrinsics_path);
    ros::param::get("/calibration_params/camera_intrinsics_path",
                    cam_intrinsics_path);
    ros::param::get("/calibration_params/imu_frame", imu_frame);
    ros::param::get("/calibration_params/lidar_frame", lidar_frame);
    ros::param::get("/calibration_params/camera_frame", camera_frame);
    ros::param::get("/calibration_params/baselink_frame", baselink_frame);
    ros::param::get("/calibration_params/world_frame", world_frame);
    ros::param::get("/calibration_params/static_extrinsics", static_extrinsics);

    if (imu_frame.empty() || lidar_frame.empty() || camera_frame.empty() ||
        baselink_frame.empty() || world_frame.empty()) {
      ROS_WARN("One or more calibration frames not set. ");
    }

    if (imu_intrinsics_path.empty()) {
      ROS_WARN("Imu intrinsics path not set. ");
    }

    if (cam_intrinsics_path.empty()) {
      ROS_WARN("Camera intrinsics path not set. ");
    }
  }

  std::string imu_intrinsics_path{};
  std::string cam_intrinsics_path{};
  std::string imu_frame{};
  std::string lidar_frame{};
  std::string camera_frame{};
  std::string baselink_frame{};
  std::string world_frame{};
  bool static_extrinsics{};
};

}  // namespace models
}  // namespace bs_parameters

#endif
