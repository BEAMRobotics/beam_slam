#pragma once

#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <ros/param.h>

#include <beam_models/parameter_base.h>

namespace beam_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct ScanMatcherParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    getParamRequired(nh, "type", type);
    getParam(nh, "queue_size", queue_size, 10);
    getParam(nh, "num_neighbors", num_neighbors, 1);
    getParamRequired(nh, "pointcloud_topic", pointcloud_topic);
    getParamRequired(nh, "initialization_path_topic",
                     initialization_path_topic);

    // need custom message for matcher_params_path
    if (!nh.getParam("matcher_params_path", matcher_params_path)) {
      const std::string info =
          "Could not find parameter matcher_params_path in namespace " +
          nh.getNamespace() +
          ", using default config in libbeam/beam_matching/config/ ";
      ROS_INFO_STREAM(info);
    }

    if (num_neighbors < 1) {
      const std::string error =
          "parameter num_neighbors must be greater than 0.";
      ROS_FATAL_STREAM(error);
      throw std::runtime_error(error);
    }

    if (queue_size < 1) {
      const std::string error = "parameter queue_size must be greater than 0.";
      ROS_FATAL_STREAM(error);
      throw std::runtime_error(error);
    }
  }

  std::string type;
  int queue_size;
  int num_neighbors;
  std::string pointcloud_topic;
  std::string initialization_path_topic;

  std::vector<size_t> position_indices;
  std::vector<size_t> orientation_indices;
};

}} // namespace beam_parameters::models
