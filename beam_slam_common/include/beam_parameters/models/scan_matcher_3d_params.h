#pragma once

#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <ros/param.h>

#include <beam_parameters/parameter_base.h>

namespace beam_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct ScanMatcher3DParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    getParamRequired<std::string>(nh, "type", type);
    getParam<int>(nh, "num_neighbors", num_neighbors, 1);
    getParam<float>(nh, "downsample_size", downsample_size, 0.03);
    getParam<double>(nh, "outlier_threshold_t", outlier_threshold_t, 0.03);
    getParam<double>(nh, "outlier_threshold_r", outlier_threshold_r, 30);
    getParam<std::string>(nh, "frame_initializer_type", frame_initializer_type,
                          frame_initializer_type);
    getParam<std::string>(nh, "frame_initializer_info", frame_initializer_info,
                          frame_initializer_info);
    getParamRequired<std::string>(nh, "pointcloud_topic", pointcloud_topic);
    getParamRequired<std::string>(nh, "pointcloud_frame", pointcloud_frame);
    getParam<bool>(nh, "fix_first_scan", fix_first_scan, false);
    getParam<std::string>(nh, "scan_output_directory", scan_output_directory,
                          "");
    nh.param("matcher_noise_diagonal", matcher_noise_diagonal, matcher_noise_diagonal);
    
    // get lag_duration from global namespace
    ros::param::get("~lag_duration", lag_duration);

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
  }

  std::vector<double> matcher_noise_diagonal{0, 0, 0, 0, 0, 0};
  std::string type;
  int num_neighbors;
  float downsample_size;
  double outlier_threshold_t;
  double outlier_threshold_r;
  double lag_duration;
  bool fix_first_scan;
  std::string pointcloud_topic;
  std::string pointcloud_frame;
  std::string frame_initializer_type{"ODOMETRY"};
  std::string frame_initializer_info{""};
  std::string matcher_params_path;
  std::string scan_output_directory;
};

}} // namespace beam_parameters::models
