#pragma once

#include <bs_parameters/parameter_base.h>
#include <fuse_loss/huber_loss.h>

#include <ros/node_handle.h>
#include <ros/param.h>

#include <string>
#include <vector>

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
    getParam<std::string>(
        nh, "visual_measurement_topic", visual_measurement_topic,
        "/local_mapper/visual_feature_tracker/visual_measurements");
    getParam<std::string>(nh, "imu_topic", imu_topic, "");
    getParam<std::string>(nh, "lidar_topic", lidar_topic, "");

    // config for an optional frame initializer
    getParam<std::string>(nh, "frame_initializer_config",
                          frame_initializer_config, frame_initializer_config);

    // path to optional output folder
    getParam<std::string>(nh, "output_folder", output_folder, output_folder);

    // mode for initializing, options: VISUAL, LIDAR
    getParam<std::string>(nh, "init_mode", init_mode, init_mode);
    if (init_mode != "VISUAL" && init_mode != "LIDAR") {
      ROS_ERROR("Invalid init mode type, options: 'VISUAL', 'LIDAR'.");
    }
    // maximum optimizaiton time in seconds
    getParam<double>(nh, "max_optimization_s", max_optimization_s, 1.0);

    // minimum parallax to intialize (if using VISUAL)
    getParam<double>(nh, "min_parallax", min_parallax, 20.0);

    // minimum acceptable trajectory length to intialize (if using frame init is
    // given or using LIDAR)
    getParam<double>(nh, "min_trajectory_length_m", min_trajectory_length_m,
                     2.0);

    // size of init window in seconds
    getParam<double>(nh, "initialization_window_s", initialization_window_s,
                     5.0);

    // sensor frequencies
    getParam<int>(nh, "imu_hz", imu_hz, 500);
    getParam<int>(nh, "lidar_hz", lidar_hz, 10);
    getParam<int>(nh, "camera_hz", camera_hz, 20);

    getParam<double>(nh, "reprojection_covariance_weight",
                     reprojection_covariance_weight, 0.01);

    double reprojection_loss_a;
    getParam<double>(nh, "reprojection_loss_a", reprojection_loss_a, 0.2);
    // reprojection loss
    reprojection_loss =
        std::make_shared<fuse_loss ::HuberLoss>(reprojection_loss_a);
  }

  std::string visual_measurement_topic{
      "/local_mapper/visual_feature_tracker/visual_measurements"};
  std::string imu_topic{""};
  std::string lidar_topic{""};
  std::string frame_initializer_config{};
  std::string init_mode{"FRAMEINIT"};
  std::string output_folder{""};

  double max_optimization_s{1.0};
  double min_parallax{20.0};
  double min_trajectory_length_m{2.0};
  double frame_init_frequency{0.1};

  double initialization_window_s{5.0};
  int imu_hz{500};
  int lidar_hz{10};
  int camera_hz{20};

  double reprojection_covariance_weight{0.01};
  fuse_core::Loss::SharedPtr reprojection_loss;
};
}} // namespace bs_parameters::models
