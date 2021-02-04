#pragma once

#include <cassert>
#include <string>
#include <vector>

#include <ceres/covariance.h>
#include <fuse_core/ceres_options.h>
#include <fuse_models/parameters/parameter_base.h>
#include <ros/console.h>
#include <ros/node_handle.h>

namespace beam_publisher { namespace parameters {

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
    nh.getParam("publish_tf", publish_tf);
    nh.getParam("predict_to_current_time", predict_to_current_time);
    nh.getParam("tf_publish_frequency", tf_publish_frequency);

    double tf_cache_time_double = tf_cache_time.toSec();
    nh.getParam("tf_cache_time", tf_cache_time_double);
    tf_cache_time.fromSec(tf_cache_time_double);

    double tf_timeout_double = tf_timeout.toSec();
    nh.getParam("tf_timeout", tf_timeout_double);
    tf_timeout.fromSec(tf_timeout_double);

    nh.getParam("queue_size", queue_size);

    nh.getParam("map_frame_id", map_frame_id);
    nh.getParam("odom_frame_id", odom_frame_id);
    nh.getParam("base_link_frame_id", base_link_frame_id);
    nh.param("base_link_output_frame_id", base_link_output_frame_id,
             base_link_frame_id);
    nh.param("world_frame_id", world_frame_id, odom_frame_id);

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

    nh.getParam("topic", topic);

    fuse_core::loadCovarianceOptionsFromROS(
        ros::NodeHandle(nh, "covariance_options"), covariance_options);
  }

  bool publish_tf{true};
  bool predict_to_current_time{false};
  double tf_publish_frequency{10.0};
  ros::Duration tf_cache_time{10.0};
  ros::Duration tf_timeout{0.1};
  int queue_size{1};
  std::string map_frame_id{"map"};
  std::string odom_frame_id{"odom"};
  std::string base_link_frame_id{"base_link"};
  std::string base_link_output_frame_id{base_link_frame_id};
  std::string world_frame_id{odom_frame_id};
  std::string topic{"odometry/filtered"};
  ceres::Covariance::Options covariance_options;
};

}} // namespace beam_publisher::parameters

#endif // FUSE_MODELS_PARAMETERS_ODOMETRY_3D_PUBLISHER_PARAMS_H
