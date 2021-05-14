#pragma once

#include <ros/node_handle.h>

#include <beam_common/sensor_config.h>

#include <beam_parameters/parameter_base.h>

namespace beam_parameters { namespace models {

struct GlobalMapperParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {}

};

}} // namespace beam_parameters::models
