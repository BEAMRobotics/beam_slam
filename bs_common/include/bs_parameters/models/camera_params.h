#ifndef bs_models_PARAMETERS_CAMERA_PARAMS_H
#define bs_models_PARAMETERS_CAMERA_PARAMS_H

#include <bs_parameters/parameter_base.h>

#include <ros/node_handle.h>
#include <ros/param.h>

#include <string>
#include <vector>

namespace bs_parameters {
namespace models {

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
  void loadFromROS(const ros::NodeHandle &nh) final {
    // subscribing topics
    getParam<std::string>(nh, "image_topic", image_topic, "");
    getParam<std::string>(nh, "init_path_topic", init_path_topic, "");
    getParam<std::string>(nh, "imu_topic", imu_topic, "");

    // vision configs
    getParam<std::string>(nh, "descriptor", descriptor, "ORB");
    getParam<std::string>(nh, "descriptor_config", descriptor_config, "");
    getParam<std::string>(nh, "detector", detector, "GFTT");
    getParam<std::string>(nh, "detector_config", detector_config, "");
    getParam<std::string>(nh, "tracker_config", tracker_config, "");

    // memory management params
    getParam<int>(nh, "window_size", window_size, 100);
    getParam<int>(nh, "keyframe_window_size", keyframe_window_size, 20);
    getParam<int>(nh, "num_features_to_track", num_features_to_track, 300);

    // keyframe decision parameters
    getParam<int>(nh, "keyframe_parallax", keyframe_parallax, 20);
    getParam<double>(nh, "keyframe_min_time_in_seconds",
                     keyframe_min_time_in_seconds, 0.2);

    // vio initialization params
    getParam<double>(nh, "init_max_optimization_time_in_seconds",
                     init_max_optimization_time_in_seconds, 0.3);
    getParam<std::string>(nh, "init_map_output_directory",
                          init_map_output_directory, "");
    getParam<bool>(nh, "init_use_scale_estimate", init_use_scale_estimate,
                   false);
  }

  // subscribing topics
  std::string image_topic{};
  std::string init_path_topic{};
  std::string imu_topic{};

  // vision configs
  std::string descriptor{};
  std::string descriptor_config{};
  std::string detector{};
  std::string detector_config{};
  std::string tracker_config{};

  // memory management params
  int window_size{};
  int keyframe_window_size{};
  int num_features_to_track{};

  // keyframe decision parameters
  int keyframe_parallax{};
  int keyframe_tracks_drop{};
  double keyframe_min_time_in_seconds{};

  // vio initialization params
  std::string init_map_output_directory{};
  double init_max_optimization_time_in_seconds{};
  bool init_use_scale_estimate{};
};
}
} // namespace bs_parameters::models

#endif
