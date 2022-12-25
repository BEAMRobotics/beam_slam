#pragma once

#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/buffer_core.h>

#include <bs_common/pose_lookup.h>

namespace bs_models { namespace frame_initializers {

static std::string frame_initializer_error_msg = "";

/**
 * @brief This base class shows the contract between a FrameInitializer class.
 * The goal of this class is to initialize the pose of a frame given some
 * timestamp. This can simply be from a published topic, or can use an odometry
 * methodology with input sensor data. For more information on frames, see the
 * PoseLookup and ExtrinsicsLookupOnline classes.
 *
 * All input data to the derived classes should be added in a custom
 * constructor. The constructor also needs to initialize pose_lookup_ and poses_
 *
 */
class FrameInitializerBase {
public:
  /**
   * @brief Gets estimated pose of sensor frame wrt world frame using
   * Poselookup.
   * @todo Change this back to non-virtual, if a request is within the current
   * path, interpolate with that, if its beyond the path, then extrapolate with
   * the source input (odom, path, transform etc)
   * Automatically choose the interpolation method (linear vs cubic) depending
   * on the frequency of the path
   * @param T_WORLD_SENSOR reference to result
   * @param time stamp of the frame being initialized
   * @param sensor_frame sensor frame id.
   * @return true if pose lookup was successful
   */
  virtual bool GetEstimatedPose(
      Eigen::Matrix4d& T_WORLD_SENSOR, const ros::Time& time,
      const std::string& sensor_frame_id,
      std::string& error_msg = frame_initializer_error_msg) = 0;

  /**
   * @brief Factory method for creating a Frame initializer from a json config
   * @param config_path path to json config
   * @return unique ptr to a frame initializer
   */
  static std::unique_ptr<frame_initializers::FrameInitializerBase>
      Create(const std::string& config_path);

  /**
   * @brief Converts incoming path message into a queue of poses
   * @param message path message
   */
  void PathCallback(const nav_msgs::PathConstPtr message);

protected:
  /**
   * @brief Sets the path callback with specified path topic and window size
   * @param path_topic ros topic path is published on
   * @param path_window_size size of path to retain
   */
  void SetPathCallback(const std::string& path_topic,
                       const int path_window_size);

  std::string authority_;
  std::shared_ptr<bs_common::PoseLookup> pose_lookup_;
  std::map<uint64_t, Eigen::Matrix4d> path_poses_;
  ros::Subscriber path_subscriber_;
  int path_window_size_;
  ros::Time path_recieved_{0.0};
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
};

}} // namespace bs_models::frame_initializers