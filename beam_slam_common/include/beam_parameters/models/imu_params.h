#ifndef BEAM_MODELS_PARAMETERS_IMU_PARAMS_H
#define BEAM_MODELS_PARAMETERS_IMU_PARAMS_H

#include <beam_parameters/parameter_base.h>

#include <ros/node_handle.h>
#include <ros/param.h>

#include <string>
#include <vector>

namespace beam_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct IMUParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    std::string ns = nh.getNamespace();
    ns.erase(0, 1);
    ns = ns.substr(0, ns.find("/"));

    ros::param::get(ns + "/imu/imu_topic", imu_topic);
    double gyro_noise, accel_noise, gyro_bias, accel_bias;
    ros::param::get(ns + "/imu/gyroscope_noise", gyro_noise);
    ros::param::get(ns + "/imu/accelerometer_noise", accel_noise);
    ros::param::get(ns + "/imu/gyroscope_bias", gyro_bias);
    ros::param::get(ns + "/imu/accelerometer_bias", accel_bias);

    cov_gyro_noise = Eigen::Matrix3d::Identity() * gyro_noise;
    cov_accel_noise = Eigen::Matrix3d::Identity() * accel_noise;
    cov_gyro_bias = Eigen::Matrix3d::Identity() * gyro_bias;
    cov_accel_bias = Eigen::Matrix3d::Identity() * accel_bias;
  }

  Eigen::Matrix3d cov_gyro_noise;
  Eigen::Matrix3d cov_accel_noise;
  Eigen::Matrix3d cov_gyro_bias;
  Eigen::Matrix3d cov_accel_bias;
  std::string imu_topic{};
};

}} // namespace beam_parameters::models

#endif
