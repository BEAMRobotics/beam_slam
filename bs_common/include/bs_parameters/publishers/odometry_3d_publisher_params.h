#pragma once

#include <ceres/covariance.h>
#include <fuse_core/ceres_options.h>
#include <ros/console.h>

#include <bs_parameters/parameter_base.h>

namespace bs_parameters { namespace publishers {

/**
 * @brief Defines the set of parameters required by the Odometry3DPublisher
 * class
 */
struct Odometry3DPublisherParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    getParam<bool>(nh, "publish_tf", publish_tf, true);
    getParam<bool>(nh, "predict_to_current_time", predict_to_current_time,
                   false);
    getParam<double>(nh, "tf_publish_frequency", tf_publish_frequency, 10);

    double tf_cache_time_double;
    getParam<double>(nh, "tf_cache_time", tf_cache_time_double, 10);
    tf_cache_time.fromSec(tf_cache_time_double);

    double tf_timeout_double;
    getParam<double>(nh, "tf_timeout", tf_timeout_double, 0.1);
    tf_timeout.fromSec(tf_timeout_double);

    getParam<int>(nh, "queue_size", queue_size, 1);
    getParam<std::string>(nh, "map_frame_id", map_frame_id, "map");
    getParam<std::string>(nh, "odom_frame_id", odom_frame_id, "odom");
    getParam<std::string>(nh, "base_link_frame_id", base_link_frame_id,
                          "base_link");
    getParam<std::string>(nh, "base_link_output_frame_id",
                          base_link_output_frame_id, base_link_frame_id);
    getParam<std::string>(nh, "world_frame_id", world_frame_id, odom_frame_id);
    getParam<std::string>(nh, "topic", topic, "odometry/filtered");

    const bool frames_valid =
        map_frame_id != odom_frame_id && map_frame_id != base_link_frame_id &&
        map_frame_id != base_link_output_frame_id &&
        odom_frame_id != base_link_frame_id &&
        odom_frame_id != base_link_output_frame_id &&
        (world_frame_id == map_frame_id || world_frame_id == odom_frame_id);

    if (!frames_valid) {
      ROS_FATAL_STREAM("Invalid frame configuration! Please note:\n"
                       << " - The values for map_frame_id, odom_frame_id, and "
                          "base_link_frame_id must be unique\n"
                       << " - The values for map_frame_id, odom_frame_id, and "
                          "base_link_output_frame_id must be unique\n"
                       << " - The world_frame_id must be the same as the "
                          "map_frame_id or odom_frame_id\n");

      assert(frames_valid);
    }

    fuse_core::loadCovarianceOptionsFromROS(
        ros::NodeHandle(nh, "covariance_options"), covariance_options);
  }

  bool publish_tf;
  bool predict_to_current_time;
  double tf_publish_frequency;
  ros::Duration tf_cache_time;
  ros::Duration tf_timeout;
  int queue_size;
  std::string map_frame_id;
  std::string odom_frame_id;
  std::string base_link_frame_id;
  std::string base_link_output_frame_id;
  std::string world_frame_id;
  std::string topic;
  ceres::Covariance::Options covariance_options;
};

}} // namespace bs_parameters::publishers
