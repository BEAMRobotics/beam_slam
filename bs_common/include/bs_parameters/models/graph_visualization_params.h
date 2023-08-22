#pragma once

#include <ros/param.h>

#include <bs_parameters/parameter_base.h>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the GraphVisualization class
 */
struct GraphVisualizationParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    getParam<bool>(nh, "publish", publish, publish);

    /** If not empty, it will output all graph info to this directory, where all
     * content will go in a new folder named using the current timestamp */
    getParam<std::string>(nh, "save_path", save_path, save_path);
  }

  std::string save_path;
  bool publish{true};
};

}} // namespace bs_parameters::models
