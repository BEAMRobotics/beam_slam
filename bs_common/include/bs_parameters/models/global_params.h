#ifndef bs_models_PARAMETERS_GLOBAL_PARAMS_H
#define bs_models_PARAMETERS_GLOBAL_PARAMS_H

#include <bs_parameters/parameter_base.h>

#include <ros/node_handle.h>
#include <ros/param.h>

#include <string>
#include <vector>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct GlobalParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    ros::param::get("~imu_intrinsics_path", imu_intrinsics_path);
    ros::param::get("~camera_intrinsics_path", cam_intrinsics_path);
    ros::param::get("~optimization_frequency", optimization_frequency);
    ros::param::get("~lag_duration", lag_duration);
    ros::param::get("~imu_frame", imu_frame);
    ros::param::get("~lidar_frame", lidar_frame);
    ros::param::get("~camera_frame", camera_frame);
    ros::param::get("~baselink_frame", baselink_frame);
    ros::param::get("~static_extrinsics", static_extrinsics);
  }

  std::string imu_intrinsics_path{};
  std::string cam_intrinsics_path{};
  double optimization_frequency{};
  double lag_duration{};
  std::string imu_frame{};
  std::string lidar_frame{};
  std::string camera_frame{};
  std::string baselink_frame{};
  bool static_extrinsics{};
};

}} // namespace bs_parameters::models

#endif
