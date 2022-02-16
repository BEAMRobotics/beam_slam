#pragma once

#include <ros/param.h>
#include <Eigen/Dense>

#include <bs_parameters/parameter_base.h>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct GTInitializerParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    // imu topic
    getParam<std::string>(nh, "imu_topic", imu_topic, "");

    /** Options: TRANSFORM, ODOMETRY, POSEFILE */
    getParam<std::string>(nh, "frame_initializer_type", frame_initializer_type,
                          frame_initializer_type);

    /** for TRANSFORM: topic, for ODOMETRY: topic, for POSEFILE: path */
    getParam<std::string>(nh, "frame_initializer_info", frame_initializer_info,
                          frame_initializer_info);

    /** Optional For Odometry frame initializer */
    getParam<std::string>(nh, "sensor_frame_id_override",
                          sensor_frame_id_override, "");

    /** Optional For Odometry or Transform frame initializer */
    std::vector<double> frame_override_tf;
    nh.param("T_ORIGINAL_OVERRIDE", frame_override_tf, frame_override_tf);
    if (frame_override_tf.size() != 16) {
      ROS_ERROR("Invalid T_ORIGINAL_OVERRIDE params, required 16 params, "
                "given: %d. Using default identity transform",
                frame_override_tf.size());
      T_ORIGINAL_OVERRIDE = Eigen::Matrix4d::Identity();
    } else {
      T_ORIGINAL_OVERRIDE = Eigen::Matrix4d(frame_override_tf.data());
    }

    // minimum trajectory length for a valid initialization
    getParam<double>(nh, "min_trajectory_length", min_trajectory_length, 0.5);

    // maximum time to wait for a valid init trajectory
    double trajectory_time_window_double;
    getParam<double>(nh, "trajectory_time_window",
                     trajectory_time_window_double, 10);
    trajectory_time_window = ros::Duration(trajectory_time_window_double);
  }

  std::string frame_initializer_type{"ODOMETRY"};
  std::string frame_initializer_info{""};
  std::string sensor_frame_id_override{};
  Eigen::Matrix4d T_ORIGINAL_OVERRIDE;
  std::string imu_topic;
  double min_trajectory_length;
  ros::Duration trajectory_time_window;
};

}} // namespace bs_parameters::models
