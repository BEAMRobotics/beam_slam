#ifndef bs_models_PARAMETERS_CAMERA_PARAMS_H
#define bs_models_PARAMETERS_CAMERA_PARAMS_H

#include <bs_parameters/parameter_base.h>

#include <ros/node_handle.h>
#include <ros/param.h>

#include <string>
#include <vector>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the ScanMatcher class
 */
struct CameraParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    getParam<std::string>(nh, "image_topic", image_topic, "");
    getParam<std::string>(nh, "source", source, "VIO");
    getParam<std::string>(nh, "init_path_topic", init_path_topic, "");
    getParam<std::string>(nh, "imu_topic", imu_topic, "");
    getParam<std::string>(nh, "init_map_output_directory",
                          init_map_output_directory, "");

    getParam<std::string>(nh, "frame_odometry_output_topic",
                          frame_odometry_output_topic, "/vio_init");
    getParam<std::string>(nh, "landmark_topic", landmark_topic,
                          "/landmarks");
    getParam<std::string>(nh, "slam_chunk_topic", slam_chunk_topic,
                          "/slam_chunks");
    getParam<std::string>(nh, "slam_chunk_topic", slam_chunk_topic,
                          "/slam_chunks");
    getParam<std::string>(nh, "descriptor", descriptor, "ORB");

    getParam<int>(nh, "window_size", window_size, 100);
    getParam<int>(nh, "keyframe_window_size", keyframe_window_size, 20);
    getParam<int>(nh, "num_features_to_track", num_features_to_track, 300);
    getParam<int>(nh, "keyframe_parallax", keyframe_parallax, 20);
    getParam<int>(nh, "keyframe_tracks_drop", keyframe_tracks_drop, 100);

    getParam<double>(nh, "keyframe_min_time_in_seconds",
                     keyframe_min_time_in_seconds, 0.3);
    getParam<double>(nh, "init_max_optimization_time_in_seconds",
                     init_max_optimization_time_in_seconds, 0.3);
  }

  std::string image_topic{};
  std::string init_path_topic{};
  std::string imu_topic{};
  std::string source{};
  std::string frame_odometry_output_topic{};
  std::string new_keyframes_topic{};
  std::string landmark_topic{};
  std::string slam_chunk_topic{};
  std::string descriptor{};
  std::string init_map_output_directory{};

  int window_size{};
  int keyframe_window_size{};
  int num_features_to_track{};
  int keyframe_parallax{};
  int keyframe_tracks_drop{};
  double keyframe_min_time_in_seconds{};
  double init_max_optimization_time_in_seconds{};
};

}} // namespace bs_parameters::models

#endif
