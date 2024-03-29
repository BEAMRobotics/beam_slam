#pragma once

#include <bs_parameters/parameter_base.h>

#include <ros/node_handle.h>
#include <ros/param.h>

namespace bs_parameters { namespace models {

struct InertialOdometryParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    getParam<double>(nh, "measurement_buffer_duration",
                     measurement_buffer_duration, measurement_buffer_duration);
    // imu topic
    getParamRequired<std::string>(nh, "imu_topic", imu_topic);

    std::string info_weights_config;
    getParamRequired<std::string>(ros::NodeHandle("~"),
                                  "information_weights_config",
                                  info_weights_config);
    nlohmann::json info_weights;
    beam::ReadJson(beam::CombinePaths(bs_common::GetBeamSlamConfigPath(),
                                      info_weights_config),
                   info_weights);
    getParamJson<double>(info_weights, "inertial_information_weight",
                         inertial_information_weight,
                         inertial_information_weight);
  }

  double measurement_buffer_duration{10.0};
  double inertial_information_weight{1.0};
  std::string imu_topic{};
};
}} // namespace bs_parameters::models
