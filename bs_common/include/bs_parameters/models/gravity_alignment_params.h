#pragma once

#include <bs_parameters/parameter_base.h>

#include <ros/node_handle.h>
#include <ros/param.h>

namespace bs_parameters { namespace models {

struct GravityAlignmentParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    // imu topic
    getParamRequired<std::string>(nh, "imu_topic", imu_topic);

    // weighting factor of inertial information matrix
    getParam<double>(nh, "gravity_info_weight", gravity_info_weight,
                     gravity_info_weight);

    // odometry topic for the poses to add constraints to
    getParamRequired<std::string>(nh, "constraint_odom_topic",
                                  constraint_odom_topic);

    // nominal direction of gravity as measured by the IMU. This is usually
    // positive or negative Z so we have implemented those two options as: '+Z'
    // or '-Z'
    getParam<std::string>(nh, "nominal_gravity_direction",
                          nominal_gravity_direction, nominal_gravity_direction);
    if (nominal_gravity_direction == "+Z") {
      gravity_nominal = {0, 0, 1};
    } else if (nominal_gravity_direction == "-Z") {
      gravity_nominal = {0, 0, -1};
    } else {
      BEAM_ERROR("Invalid nominal_gravity_direction param, supported options "
                 "are: +Z, -Z");
      throw std::invalid_argument{"invalid nominal_gravity_direction param"};
    }
  }

  double measurement_buffer_duration{10.0};
  double gravity_info_weight{1.0};
  std::string imu_topic{};
  std::string constraint_odom_topic{};
  std::string nominal_gravity_direction{"+Z"};
  Eigen::Vector3d gravity_nominal;
};
}} // namespace bs_parameters::models
