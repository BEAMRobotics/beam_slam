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
    gyro_noise_covariance = fuse_core::getCovarianceDiagonalParam<3>(
        nh, "accel_noise_covariance", 1.5e-03);    
    accel_noise_covariance = fuse_core::getCovarianceDiagonalParam<3>(
        nh, "accel_noise_covariance", 4.0e-03);
    gyro_bias_covariance = fuse_core::getCovarianceDiagonalParam<3>(
        nh, "accel_noise_covariance", 3.5e-05);        
    accel_bias_covariance = fuse_core::getCovarianceDiagonalParam<3>(
        nh, "accel_noise_covariance", 6.5e-05);
  }

  int queue_size{50};
  double gravitational_acceleration{9.80665};
  fuse_core::Matrix3d gyro_noise_covariance;  
  fuse_core::Matrix3d accel_noise_covariance;
  fuse_core::Matrix3d gyro_bias_covariance;
  fuse_core::Matrix3d accel_bias_covariance;
};

}}  // namespace beam_parameters::models
