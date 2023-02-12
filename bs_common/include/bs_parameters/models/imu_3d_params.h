#pragma once

#include <fuse_core/parameter.h>
#include <numeric>
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
    nh.getParam("frame_initializer_config", frame_initializer_config);

    std::vector<double> prior_diagonal;
    nh.param("frame_initializer_prior_noise_diagonal", prior_diagonal,
             prior_diagonal);
    if (prior_diagonal.size() != 6) {
      ROS_ERROR("Invalid gm_noise_diagonal params, required 6 params, "
                "given: %zu. Using default (0.1 for all)",
                prior_diagonal.size());
      prior_diagonal = std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    }
    if (std::accumulate(prior_diagonal.begin(), prior_diagonal.end(), 0.0) ==
        0.0) {
      ROS_INFO("Prior diagonal set to zero, not adding priors");
      use_pose_priors = false;
    }
    for (int i = 0; i < 6; i++) { prior_covariance(i, i) = prior_diagonal[i]; }
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
  std::string frame_initializer_config{""};
  Eigen::Matrix<double, 6, 6> prior_covariance{
      Eigen::Matrix<double, 6, 6>::Identity()};
  bool use_pose_priors{true};
};

}} // namespace bs_parameters::models
