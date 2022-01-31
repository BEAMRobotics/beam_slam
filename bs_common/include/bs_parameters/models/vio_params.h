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
struct VioParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    /** subscribing topics */
    getParam<std::string>(nh, "image_topic", image_topic, "");
    getParam<std::string>(nh, "imu_topic", imu_topic, "");

    /** vision configs */
    getParam<std::string>(nh, "descriptor", descriptor, "ORB");
    getParam<std::string>(nh, "detector", detector, "FAST");
    getParam<std::string>(nh, "descriptor_config", descriptor_config, "");
    getParam<std::string>(nh, "detector_config", detector_config, "");
    getParam<std::string>(nh, "tracker_config", tracker_config, "");

    /** memory management params */

    // number of images to store landmark information in tracker
    getParam<int>(nh, "tracker_window_size", tracker_window_size, 100);

    // number of keyframes to store before publishing slam chunk
    getParam<int>(nh, "keyframe_window_size", keyframe_window_size, 20);

    // number of features per keyframe to store in local visual map (should
    // match whats in detector config)
    getParam<int>(nh, "num_features_to_track", num_features_to_track, 300);

    /** keyframe decision parameters*/

    // if avg parallax to the previous keyframe exceeds this then set keyframe
    // (NOT USED) getParam<int>(nh, "keyframe_parallax", keyframe_parallax, 20);

    // minimum amount of time between keyframes
    getParam<double>(nh, "keyframe_parallax", keyframe_parallax, 10.0);

    /** vio initialization params */

    // max amount of time to optimize the initializer map
    getParam<double>(nh, "init_max_optimization_time_in_seconds",
                     init_max_optimization_time_in_seconds, 0.3);

    // Outputs vio initial map into directory if not empty
    getParam<std::string>(nh, "init_map_output_directory",
                          init_map_output_directory, "");

    // Set to true if using sfm initializer
    getParam<bool>(nh, "init_use_scale_estimate", init_use_scale_estimate,
                   false);

    getParam<double>(nh, "reloc_request_period", reloc_request_period, 3.0);

    /** Options: TRANSFORM, ODOMETRY, POSEFILE */
    getParam<std::string>(nh, "frame_initializer_type", frame_initializer_type,
                          frame_initializer_type);

    /** for TRANSFORM: topic, for ODOMETRY: topic, for POSEFILE: path */
    getParam<std::string>(nh, "frame_initializer_info", frame_initializer_info,
                          frame_initializer_info);

    /** Optional For Odometry frame initializer */
    getParam<std::string>(nh, "frame_initializer_sensor_frame_id",
                          frame_initializer_sensor_frame_id, "");

    std::vector<double> prior_diagonal;
    nh.param("frame_initializer_prior_noise_diagonal", prior_diagonal,
             prior_diagonal);
    if (prior_diagonal.size() != 6) {
      ROS_ERROR("Invalid gm_noise_diagonal params, required 6 params, "
                "given: %d. Using default (1e-9).",
                prior_diagonal.size());
      prior_diagonal = std::vector<double>{1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9};
    }
    if (std::accumulate(prior_diagonal.begin(), prior_diagonal.end(), 0.0) ==
        0.0) {
      ROS_INFO("Prior diagonal set to zero, not adding priors");
      use_pose_priors = false;
    }
    for (int i = 0; i < 6; i++) { prior_covariance(i, i) = prior_diagonal[i]; }
  }

  // subscribing topics
  std::string image_topic{};
  std::string imu_topic{};

  // frame initializer
  std::string frame_initializer_type{""};
  std::string frame_initializer_info{""};
  std::string frame_initializer_sensor_frame_id{};
  Eigen::Matrix<double, 6, 6> prior_covariance{
      Eigen::Matrix<double, 6, 6>::Identity()};
  bool use_pose_priors{true};

  // vision configs
  std::string descriptor{};
  std::string descriptor_config{};
  std::string detector{};
  std::string detector_config{};
  std::string tracker_config{};

  // memory management params
  int tracker_window_size{};
  int keyframe_window_size{};
  int num_features_to_track{};
  double reloc_request_period{3.0};

  // keyframe decision parameters
  double keyframe_parallax{10.0};

  // vio initialization params
  std::string init_map_output_directory{};
  double init_max_optimization_time_in_seconds{};
  bool init_use_scale_estimate{};
};
}} // namespace bs_parameters::models
