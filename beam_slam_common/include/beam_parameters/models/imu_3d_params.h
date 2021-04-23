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
    nh.getParam("buffer_size", buffer_size);
    nh.getParam("gravitational_acceleration", gravitational_acceleration);
    nh.getParam("initial_imu_acceleration_bias", initial_imu_acceleration_bias);
    nh.getParam("initial_imu_gyroscope_bias", initial_imu_gyroscope_bias);
    imu_noise_covariance = fuse_core::getCovarianceDiagonalParam<6>(
        nh, "imu_noise_covariance", 0.0);
  }

  int queue_size{50};
  int buffer_size{50};
  double gravitational_acceleration{9.80665};
  double initial_imu_acceleration_bias{1.0e-05};
  double initial_imu_gyroscope_bias{1.0e-05};
  fuse_core::Matrix6d imu_noise_covariance;
};

}}  // namespace beam_parameters::models
