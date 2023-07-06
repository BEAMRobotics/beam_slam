#ifndef bs_models_PARAMETERS_GLOBAL_PARAMS_H
#define bs_models_PARAMETERS_GLOBAL_PARAMS_H

#include <string>
#include <vector>

#include <ros/node_handle.h>
#include <ros/param.h>

#include <beam_utils/filesystem.h>

#include <bs_parameters/parameter_base.h>
#include <bs_common/utils.h>

namespace bs_parameters { namespace models {

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
    std::string imu_intrinsics_path_rel;
    ros::param::get("/calibration_params/imu_intrinsics_path",
                    imu_intrinsics_path_rel);
    std::string cam_intrinsics_path_rel;
    ros::param::get("/calibration_params/camera_intrinsics_path",
                    cam_intrinsics_path_rel);
    ros::param::get("/calibration_params/imu_frame", imu_frame);
    ros::param::get("/calibration_params/lidar_frame", lidar_frame);
    ros::param::get("/calibration_params/camera_frame", camera_frame);
    ros::param::get("/calibration_params/baselink_frame", baselink_frame);
    ros::param::get("/calibration_params/world_frame", world_frame);
    ros::param::get("/calibration_params/static_extrinsics", static_extrinsics);

    ros::param::get("/calibration_params/camera_hz", camera_hz);
    ros::param::get("/calibration_params/imu_hz", imu_hz);
    ros::param::get("/calibration_params/lidar_hz", lidar_hz);

    if (imu_frame.empty() || lidar_frame.empty() || camera_frame.empty() ||
        baselink_frame.empty() || world_frame.empty()) {
      ROS_WARN("One or more calibration frames not set. ");
    }

    if (imu_intrinsics_path_rel.empty()) {
      ROS_WARN("Imu intrinsics path not set. ");
    } else {
      imu_intrinsics_path = beam::CombinePaths(
          bs_common::GetBeamSlamCalibrationsPath(), imu_intrinsics_path_rel);
    }

    if (cam_intrinsics_path_rel.empty()) {
      ROS_WARN("Camera intrinsics path not set. ");
    } else {
      cam_intrinsics_path = beam::CombinePaths(
          bs_common::GetBeamSlamCalibrationsPath(), cam_intrinsics_path_rel);
    }
  }

  std::string imu_intrinsics_path{};
  std::string cam_intrinsics_path{};
  std::string imu_frame{};
  std::string lidar_frame{};
  std::string camera_frame{};
  std::string baselink_frame{};
  std::string world_frame{};
  int camera_hz;
  int imu_hz;
  int lidar_hz;
  bool static_extrinsics{};
};

}} // namespace bs_parameters::models

#endif
