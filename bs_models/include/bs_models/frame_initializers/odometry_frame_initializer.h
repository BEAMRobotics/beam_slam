#pragma once

#include <nav_msgs/Odometry.h>

#include <bs_models/frame_initializers/frame_initializer_base.h>

namespace bs_models { namespace frame_initializers {

/**
 * @brief This class can be used to estimate a pose of a frame given its
 * timestamp. This is done by building a tf tree with incoming odometry messages
 * then looking up the transform at the given time.
 */
class OdometryFrameInitializer : public FrameInitializerBase {
public:
  /**
   * @brief Constructor
   * @param topic odometry topic to subscribe to
   * @param queue_size subscriber queue size
   * @param poses_buffer_time length of time (in seconds) to store poses for
   * interpolation
   * @param sensor_frame_id frame ID attached to the sensor. If this is set, it
   * will override the sensor_frame in the odometry message
   * @param T_ORIGINAL_OVERRIDE transform from the original frame in the
   * transform message to the overidden frame id
   *
   * The frame initializer takes transforms from an odometry message to build a
   * trajectory that frame poses can be sampled from. Since our odometry message
   * might have poses in some frame other than the SLAM baselink frame, we
   * provide a way to convert the poses of the odometry message into the
   * baselink pose. If the pose is in the IMU, Camera, or Lidar frame, then we
   * can use the extrinsics to look up the baselink pose for each odometry
   * message. If some other sensor frame is used, (e.g., vicon poses) then you
   * can provide a transform from the SLAM baselink frame to the override frame.
   */
  OdometryFrameInitializer(
      const std::string& topic, int queue_size, int64_t poses_buffer_time,
      const std::string& sensor_frame_id_override = "",
      const Eigen::Matrix4d& T_ORIGINAL_OVERRIDE = Eigen::Matrix4d::Identity());

  /**
   * @brief Converts incoming odometry messages to tf poses and stores them in a
   * buffercore
   * @param message odometry message
   */
  void OdometryCallback(const nav_msgs::OdometryConstPtr message);

private:
  /**
   * @brief Check to see if world frame and baselink frame IDs match those
   * supplied in odometry messages.
   */
  void CheckOdometryFrameIDs(const nav_msgs::OdometryConstPtr message);

  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  std::shared_ptr<tf2::BufferCore> poses_;

  Eigen::Matrix4d T_ORIGINAL_OVERRIDE_{};

  ros::Subscriber odometry_subscriber_;
  bool check_world_baselink_frames_{true};
  bool override_sensor_frame_id_{false};
  std::string sensor_frame_id_;
};

}} // namespace bs_models::frame_initializers