#pragma once

#include <nav_msgs/Odometry.h>
#include <tf2/buffer_core.h>

#include <beam_common/pose_lookup.h>
#include <beam_models/frame_initializers/frame_initializer_base.h>

namespace beam_models { namespace frame_initializers {

/**
 * @brief This class can be used to estimate a pose of a frame given its
 * timestamp. This is done by building a tf tree with incoming odometry messages
 * then looking up the transform at the given time. For more information on the
 * frames, see the PoseLookup class.
 *
 */
class OdometryFrameInitializer : public FrameInitializerBase {
public:
  /**
   * @brief Constructor
   * @param topic odometry topic to subscribe to
   * @param queue_size subscriber queue size
   * @param sensor_frame_id frame ID attached to the sensor, used to lookup
   * extrinsic calibrations. If not supplied, it will assume the odometry is
   * already in the correct frame.
   * @param static_extrinsics set to false if extrinsics change and transforms
   * are broadcasted to /tf
   * @param poses_buffer_time lenth of time to store poses for interpolation
   */
  OdometryFrameInitializer(const std::string& topic, int queue_size,
                           const std::string& sensor_frame_id,
                           bool static_extrinsics, double poses_buffer_time);

  /**
   * @brief Gets estimate frame pose
   * @param time stamp of the frame being initialized
   * @param T_WORLD_SENSOR reference to result
   * @return true if pose lookup was successful
   */
  bool GetEstimatedPose(const ros::Time& time,
                        Eigen::Matrix4d& T_WORLD_SENSOR) override;

  /**
   * @brief Converts incoming odometry messages to tf poses and stores them in a
   * buffercore
   * @param message odometry message
   */
  void OdometryCallback(const nav_msgs::OdometryConstPtr message);

private:
  beam_common::PoseLookup::Params pose_lookup_params_;
  std::unique_ptr<beam_common::PoseLookup> pose_lookup_;
  std::shared_ptr<tf2::BufferCore> poses_;
  ros::Subscriber odometry_subscriber_;
};

}} // namespace beam_models::frame_initializers