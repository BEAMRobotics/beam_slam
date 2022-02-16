#pragma once

#include <fuse_core/parameter.h>
#include <ros/param.h>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the Imu3D class
 */
struct Imu3DParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    // Required
    fuse_core::getParamRequired(nh, "input_topic", input_topic);
    fuse_core::getParamRequired(nh, "init_velocity_x", init_velocity_x);
    fuse_core::getParamRequired(nh, "init_velocity_y", init_velocity_y);
    fuse_core::getParamRequired(nh, "init_velocity_z", init_velocity_z);
    fuse_core::getParamRequired(nh, "init_gyro_bias", init_gyro_bias);
    fuse_core::getParamRequired(nh, "init_accel_bias", init_accel_bias);

    // Optional
    nh.getParam("queue_size", queue_size);
    nh.getParam("key_frame_rate", key_frame_rate);
    nh.getParam("lag_duration", lag_duration);
    nh.getParam("cov_prior_noise", cov_prior_noise);
    nh.getParam("frame_initializer_type", frame_initializer_type);
    nh.getParam("frame_initializer_info", frame_initializer_info);
    nh.getParam("sensor_frame_id_override", sensor_frame_id_override);

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
  }

  std::string input_topic;
  double init_velocity_x;
  double init_velocity_y;
  double init_velocity_z;
  double init_gyro_bias;
  double init_accel_bias;

  int queue_size{300};
  double key_frame_rate{1.0};
  double lag_duration{1.0};
  double cov_prior_noise{1e-9};
  std::string frame_initializer_type{"ODOMETRY"};
  std::string frame_initializer_info{""};
  std::string sensor_frame_id_override{""};
  Eigen::Matrix4d T_ORIGINAL_OVERRIDE;
};

}} // namespace bs_parameters::models
