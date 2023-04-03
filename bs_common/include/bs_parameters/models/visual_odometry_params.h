#pragma once

#include <bs_parameters/parameter_base.h>

#include <ros/node_handle.h>
#include <ros/param.h>

#include <string>
#include <vector>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct VisualOdometryParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    /** subscribing topics */
    getParam<std::string>(
        nh, "visual_measurement_topic", visual_measurement_topic,
        "/local_mapper/visual_feature_tracker/visual_measurements");

    // config for an optional frame initializer
    getParam<std::string>(nh, "frame_initializer_config",
                          frame_initializer_config, frame_initializer_config);

    // frame initializer priors
    getParam<bool>(nh, "use_pose_priors", use_pose_priors, false);
    std::vector<double> prior_diagonal;
    nh.param("frame_initializer_prior_noise_diagonal", prior_diagonal,
             prior_diagonal);
    if (prior_diagonal.size() != 6) {
      ROS_ERROR("Invalid noise_diagonal params, required 6 params, "
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

    // feature tracker container size
    getParam<int>(nh, "max_container_size", max_container_size, 300);

    // period in which to perform reloc requests
    getParam<double >(nh, "reloc_request_period", reloc_request_period, 1.0);

    // keyframe decision parameters
    getParam<double >(nh, "keyframe_parallax", keyframe_parallax, 10.0);
    getParam<double >(nh, "keyframe_translation_m", rkeyframe_translation_m, 1.0);
    getParam<double >(nh, "keyframe_rotation_deg", keyframe_rotation_deg, 1.0);
  }

  std::string visual_measurement_topic{
      "/local_mapper/visual_feature_tracker/visual_measurements"};
  int max_container_size{300};
  std::string frame_initializer_config{""};
  Eigen::Matrix<double, 6, 6> prior_covariance{
      Eigen::Matrix<double, 6, 6>::Identity()};
  bool use_pose_priors{false};
  double reloc_request_period{3.0};
  double keyframe_parallax{10.0};
  double keyframe_translation_m{2.0};
  double keyframe_rotation_deg{20.0};
};
}} // namespace bs_parameters::models
