#pragma once

#include <fuse_core/parameter.h>
#include <fuse_models/parameters/parameter_base.h>
#include <ros/param.h>

namespace bs_parameters {
namespace models {

using namespace fuse_models::parameters;

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

    // Optional
    nh.getParam("queue_size", queue_size);
    nh.getParam("key_frame_rate", key_frame_rate);
    nh.getParam("prior_noise", prior_noise);
    nh.getParam("frame_initializer_type", frame_initializer_type);
    nh.getParam("frame_initializer_info", frame_initializer_info);
    nh.getParam("sensor_frame_id_override", sensor_frame_id_override);
    cov_gyro_noise = fuse_core::getCovarianceDiagonalParam<3>(
        nh, "gyro_noise_covariance", 1e-4);
    cov_accel_noise = fuse_core::getCovarianceDiagonalParam<3>(
        nh, "accel_noise_covariance", 1e-3);
    cov_gyro_bias = fuse_core::getCovarianceDiagonalParam<3>(
        nh, "gyro_bias_covariance", 1e-6);
    cov_accel_bias = fuse_core::getCovarianceDiagonalParam<3>(
        nh, "accel_bias_covariance", 1e-4);
  }

  std::string input_topic;

  int queue_size{300};
  double key_frame_rate{1.0};
  double prior_noise{1e-9};
  std::string frame_initializer_type{"ODOMETRY"};
  std::string frame_initializer_info{""};
  std::string sensor_frame_id_override{""};
  fuse_core::Matrix3d cov_gyro_noise;
  fuse_core::Matrix3d cov_accel_noise;
  fuse_core::Matrix3d cov_gyro_bias;
  fuse_core::Matrix3d cov_accel_bias;
};

}  // namespace models
}  // namespace bs_parameters
