#pragma once

#include <beam_parameters/models/frame_to_frame_parameter_base.h>

namespace beam_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the Imu3D class
 */
struct Imu3DParams : public FrameToFrameParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadExtraParams(const ros::NodeHandle& nh) final {
    nh.getParam("queue_size", queue_size);
    nh.getParam("gravitational_acceleration", gravitational_acceleration);
    nh.getParam("prior_noise", prior_noise);
    cov_gyro_noise = fuse_core::getCovarianceDiagonalParam<3>(
        nh, "accel_noise_covariance", 1.5e-03);    
    cov_accel_noise = fuse_core::getCovarianceDiagonalParam<3>(
        nh, "accel_noise_covariance", 4.0e-03);
    cov_gyro_bias = fuse_core::getCovarianceDiagonalParam<3>(
        nh, "accel_noise_covariance", 3.5e-05);        
    cov_accel_bias = fuse_core::getCovarianceDiagonalParam<3>(
        nh, "accel_noise_covariance", 6.5e-05);
  }

  int queue_size{300};
  double gravitational_acceleration{9.80665};
  double prior_noise{1e-9};
  fuse_core::Matrix3d cov_gyro_noise;  
  fuse_core::Matrix3d cov_accel_noise;
  fuse_core::Matrix3d cov_gyro_bias;
  fuse_core::Matrix3d cov_accel_bias;
};

}}  // namespace beam_parameters::models
