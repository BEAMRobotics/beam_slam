#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <fuse_loss/cauchy_loss.h>
#include <ros/node_handle.h>
#include <ros/param.h>

#include <bs_common/utils.h>
#include <bs_parameters/parameter_base.h>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct SLAMInitializationParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    // subscribing topics
    getParam<std::string>(nh, "imu_topic", imu_topic, "");
    getParam<std::string>(nh, "lidar_topic", lidar_topic, "");

    // config for an optional frame initializer
    std::string frame_initializer_config_rel;
    getParam<std::string>(nh, "frame_initializer_config",
                          frame_initializer_config_rel,
                          frame_initializer_config_rel);
    if (!frame_initializer_config_rel.empty()) {
      frame_initializer_config = beam::CombinePaths(
          bs_common::GetBeamSlamConfigPath(), frame_initializer_config_rel);
    }

    // path to optional output folder
    getParam<std::string>(nh, "output_folder", output_folder, output_folder);

    // mode for initializing, options: VISUAL, LIDAR
    getParam<std::string>(nh, "init_mode", init_mode, init_mode);
    if (init_mode != "VISUAL" && init_mode != "LIDAR") {
      ROS_ERROR("Invalid init mode type, options: 'VISUAL', 'LIDAR'.");
    }

    // maximum optimizaiton time in seconds
    getParam<double>(nh, "max_optimization_s", max_optimization_s, 1.0);

    // minimum acceptable trajectory length to intialize (if using frame init is
    // given or using LIDAR)
    getParam<double>(nh, "min_trajectory_length_m", min_trajectory_length_m,
                     min_trajectory_length_m);

    // minimum acceptable parallax to intialize (if using frame init is
    // given or using VISUAL)
    getParam<double>(nh, "min_visual_parallax", min_visual_parallax,
                     min_visual_parallax);

    // size of init window in seconds. This controls the data buffers, this
    // should be larger than the amount of time it takes to produce the min.
    // traj.
    getParam<double>(nh, "initialization_window_s", initialization_window_s,
                     initialization_window_s);

    std::string matcher_config_rel;
    getParam<std::string>(nh, "matcher_config", matcher_config_rel,
                          matcher_config_rel);
    if (!matcher_config_rel.empty()) {
      matcher_config = beam::CombinePaths(bs_common::GetBeamSlamConfigPath(),
                                          matcher_config_rel);
    }

    /// Load all information matrix weights for the optimization problem
    std::string info_weights_config;

    getParamRequired<std::string>(ros::NodeHandle("~"),
                                  "information_weights_config",
                                  info_weights_config);
    nlohmann::json info_weights;

    std::string info_weights_path = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), info_weights_config);
    ROS_INFO("reading info weights from from: %s", info_weights_path.c_str());
    if (!beam::ReadJson(info_weights_path, info_weights)) {
      ROS_ERROR("cannot read info weights json, using defaults");
    } else {
      getParamJson<double>(info_weights, "inertial_information_weight",
                           inertial_information_weight,
                           inertial_information_weight);
      getParamJson<double>(info_weights, "lidar_information_weight",
                           lidar_information_weight, lidar_information_weight);
      getParamJson<double>(info_weights, "reprojection_information_weight",
                           reprojection_information_weight,
                           reprojection_information_weight);
    }

    // reprojection loss
    double reprojection_loss_a = 5.0 * reprojection_information_weight;
    reprojection_loss =
        std::make_shared<fuse_loss::CauchyLoss>(reprojection_loss_a);

    // read vo params
    std::string vo_config;
    getParam<std::string>(nh, "vo_config", vo_config, vo_config);
    readVOParams(vo_config);
  }

  /**
   * @brief Reads visual odom params from a json file
   *
   * @param[in] vo_config string of the vo config under the config folder
   */
  void readVOParams(const std::string& vo_config) {
    if (vo_config.empty()) {
      ROS_ERROR("Empty VO config path, using default params.");
      return;
    }

    std::string vo_params =
        beam::CombinePaths(bs_common::GetBeamSlamConfigPath(), vo_config);
    nlohmann::json J;

    ROS_INFO("reading vo params from: %s", vo_params.c_str());
    if (!beam::ReadJson(vo_params, J)) {
      ROS_ERROR("Invalid VO config path, using default params.");
      return;
    }

    getParamJson<bool>(J, "use_idp", use_idp, use_idp);
    getParamJson<double>(J, "max_triangulation_distance",
                         max_triangulation_distance,
                         max_triangulation_distance);
    getParamJson<double>(J, "max_triangulation_reprojection",
                         max_triangulation_reprojection,
                         max_triangulation_reprojection);
    getParamJson<double>(J, "track_outlier_pixel_threshold",
                         track_outlier_pixel_threshold,
                         track_outlier_pixel_threshold);
    getParamJson<bool>(J, "local_map_matching", local_map_matching,
                       local_map_matching);
    getParamJson<bool>(J, "use_online_calibration", use_online_calibration,
                       use_online_calibration);
  }

  std::string visual_measurement_topic{"/feature_tracker/visual_measurements"};
  std::string imu_topic{""};
  std::string lidar_topic{""};
  std::string frame_initializer_config{""};
  std::string init_mode{"FRAMEINIT"};
  std::string output_folder{""};

  std::string matcher_config;
  double max_optimization_s{1.0};

  // optimization weights
  double inertial_information_weight{1.0};
  double reprojection_information_weight{1.0};
  double lidar_information_weight{1.0};

  // slam init thresholds
  double min_trajectory_length_m{2.0};
  double min_visual_parallax{40.0};
  double frame_init_frequency{0.1};
  double initialization_window_s{10.0};

  // vo params
  bool use_online_calibration{false};
  bool use_idp{false};
  bool local_map_matching{false};
  double max_triangulation_distance{30.0};
  double max_triangulation_reprojection{40.0};
  double track_outlier_pixel_threshold{1.0};
  fuse_core::Loss::SharedPtr reprojection_loss;
};
}} // namespace bs_parameters::models
