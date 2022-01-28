#pragma once

#include <geometry_msgs/TransformStamped.h>

#include <bs_models/frame_initializers/frame_initializer_base.h>

namespace bs_models { namespace frame_initializers {

/**
 * @brief This class can be used to estimate a pose of a frame given its
 * timestamp. This is done by building a tf tree with incoming odometry messages
 * then looking up the transform at the given time.
 */
class TransformFrameInitializer : public FrameInitializerBase {
public:
  /**
   * @brief Constructor
   * @param topic odometry topic to subscribe to
   * @param queue_size subscriber queue size
   * @param poses_buffer_time length of time (in seconds) to store poses for
   * interpolation
   * @param sensor_frame_id frame ID attached to the sensor. If this is set, it
   * will override the sensor_frame in the transform message
   */
  TransformFrameInitializer(const std::string& topic, int queue_size,
                           int64_t poses_buffer_time,
                           const std::string& sensor_frame_id_override = "");

  /**
   * @brief Converts incoming odometry messages to tf poses and stores them in a
   * buffercore
   * @param message odometry message
   */
  void TransformCallback(const geometry_msgs::TransformStampedConstPtr message);

private:
  /**
   * @brief Check to see if world frame and baselink frame IDs match those
   * supplied in odometry messages.
   */
  void CheckTransformFrameIDs(const geometry_msgs::TransformStampedConstPtr message);

  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  std::shared_ptr<tf2::BufferCore> poses_;

  ros::Subscriber transform_subscriber_;
  bool check_world_baselink_frames_{true};
  bool override_sensor_frame_id_{false};
  std::string sensor_frame_id_;
};

}} // namespace bs_models::frame_initializers