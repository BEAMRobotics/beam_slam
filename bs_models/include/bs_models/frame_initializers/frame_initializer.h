#pragma once

#include <Eigen/Dense>
#include <mutex>
#include <ros/ros.h>
#include <tf2/buffer_core.h>

#include <bs_common/pose_lookup.h>

namespace bs_models {

static std::string frame_initializer_error_msg = "";

using Path = std::map<ros::Time, Eigen::Matrix4d>;

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
class FrameInitializer {
public:
  /**
   * @brief Constructor to create frame initializer from a config file
   * @param config_path path to config file
   */
  FrameInitializer(const std::string& config_path);

  /**
   * @brief Gets estimated pose of sensor frame wrt world frame using
   * Poselookup.
   * @param T_WORLD_SENSOR reference to result
   * @param time stamp of the frame being initialized
   * @param sensor_frame sensor frame id.
   * @return true if pose lookup was successful
   */
  bool GetPose(Eigen::Matrix4d& T_WORLD_SENSOR, const ros::Time& time,
               const std::string& sensor_frame_id,
               std::string& error_msg = frame_initializer_error_msg);

  /**
   * @brief Gets relative pose between two timestamps wrt the world frame
   * @param T_A_B [out] relative pose
   * @param tA [in] time at state A
   * @param tB [in] time at state B
   * @return true if pose lookup was successful
   */
  bool GetRelativePose(Eigen::Matrix4d& T_A_B, const ros::Time& tA,
                       const ros::Time& tB,
                       std::string& error_msg = frame_initializer_error_msg);

  /**
   * @brief Converts incoming odometry messages to tf poses and stores them in a
   * buffercore
   * @param message odometry message
   */
  void OdometryCallback(const nav_msgs::OdometryConstPtr message);

  /**
   * @brief Stores the current path published from the graph
   * @param message path message
   */
  void PathCallback(const nav_msgs::PathConstPtr message);

  /**
   * @brief Clears any local data that is stored
   */
  void Clear();

private:
  /**
   * @brief Check to see if world frame and baselink frame IDs match those
   * supplied in odometry messages.
   */
  void CheckOdometryFrameIDs(const nav_msgs::OdometryConstPtr message);

  /**
   * @brief Initializes the class from a pose file
   */
  void InitializeFromPoseFile(const std::string& file_path);

  std::string authority_;
  std::shared_ptr<bs_common::PoseLookup> pose_lookup_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  std::shared_ptr<tf2::BufferCore> poses_;

  Eigen::Matrix4d T_ORIGINAL_OVERRIDE_{};

  Path graph_path_;
  ros::Duration poses_buffer_duration_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber path_subscriber_;
  bool check_world_baselink_frames_{true};
  bool override_sensor_frame_id_{false};
  std::string sensor_frame_id_;
  std::mutex path_mutex_;
  std::string type_;
};

} // namespace bs_models