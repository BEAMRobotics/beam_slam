#include <beam_models/frame_initializers/odometry_frame_initializer.h>

#include <beam_utils/log.h>

#include <beam_common/utils.h>

namespace beam_models {
namespace frame_initializers {

OdometryFrameInitializer::OdometryFrameInitializer(
    const std::string& topic, int queue_size, int64_t poses_buffer_time,
    const std::string& sensor_frame_id_override) {
  poses_ = std::make_shared<tf2::BufferCore>(ros::Duration(poses_buffer_time));
  pose_lookup_ = std::make_shared<beam_common::PoseLookup>(poses_);

  ros::NodeHandle n;
  odometry_subscriber_ = n.subscribe<nav_msgs::Odometry>(
      topic, queue_size,
      boost::bind(&OdometryFrameInitializer::OdometryCallback, this, _1));

  if (!sensor_frame_id_override.empty()) {
    if (sensor_frame_id_override != extrinsics_.GetImuFrameId() &&
        sensor_frame_id_override != extrinsics_.GetCameraFrameId() &&
        sensor_frame_id_override != extrinsics_.GetLidarFrameId()) {
      BEAM_ERROR(
          "Sensor frame id override provided does not match any frame in the "
          "extrinsics. Input: {}",
          sensor_frame_id_override);
      throw std::invalid_argument{"Invalid sensor frame id override."};
    } else {
      BEAM_INFO("Overriding sensor frame id in odometry messages to: {}",
                sensor_frame_id_override);
      sensor_frame_id_ = sensor_frame_id_override;
      override_sensor_frame_id_ = true;
    }
  } else {
    sensor_frame_id_ = extrinsics_.GetBaselinkFrameId();
  }
}

void OdometryFrameInitializer::CheckOdometryFrameIDs(
    const nav_msgs::OdometryConstPtr message) {
  check_world_baselink_frames_ = false;

  std::string parent_frame_id = message->header.frame_id;
  std::string child_frame_id = message->child_frame_id;

  if (parent_frame_id.find(pose_lookup_->GetWorldFrameId()) ==
      std::string::npos) {
    BEAM_WARN(
        "World frame in extrinsics does not match parent frame in odometry "
        "messages. Using extrinsics.");
  }

  // if we've overriden the sensor frame id, then we've already checked its
  // valid. If not, we need to check that the odometry messages are valid.
  if (!override_sensor_frame_id_) {
    if (child_frame_id.find(extrinsics_.GetImuFrameId()) != std::string::npos) {
      sensor_frame_id_ = extrinsics_.GetImuFrameId();
    } else if (child_frame_id.find(extrinsics_.GetCameraFrameId()) !=
               std::string::npos) {
      sensor_frame_id_ = extrinsics_.GetCameraFrameId();
    } else if (child_frame_id.find(extrinsics_.GetLidarFrameId()) !=
               std::string::npos) {
      sensor_frame_id_ = extrinsics_.GetLidarFrameId();
    } else {
      BEAM_WARN(
          "Sensor frame id in odometry message not equal to any sensor frame "
          "in extrinsics. Please provide a sensor_frame_id_override.");
      throw std::invalid_argument{"Invalid frame id"};
    }
  }
}

void OdometryFrameInitializer::OdometryCallback(
    const nav_msgs::OdometryConstPtr message) {
  if (check_world_baselink_frames_) {
    CheckOdometryFrameIDs(message);
  }

  // if sensor_frame is already baselink, then we can directly copy
  if (sensor_frame_id_ == extrinsics_.GetBaselinkFrameId()) {
    geometry_msgs::TransformStamped tf_stamped;
    beam_common::OdometryMsgToTransformedStamped(
        *message, message->header.stamp, message->header.seq,
        extrinsics_.GetWorldFrameId(), sensor_frame_id_, tf_stamped);
    std::string authority{"odometry"};
    poses_->setTransform(tf_stamped, authority, false);
    return;
  }

  // Otherwise, we need to apply the extrinsics
  Eigen::Matrix4d T_SENSORFRAME_BASELINK;
  bool lookup_success;
  if (sensor_frame_id_ == extrinsics_.GetImuFrameId()) {
    lookup_success = extrinsics_.GetT_IMU_BASELINK(T_SENSORFRAME_BASELINK,
                                                   message->header.stamp);
  } else if (sensor_frame_id_ == extrinsics_.GetCameraFrameId()) {
    lookup_success = extrinsics_.GetT_CAMERA_BASELINK(T_SENSORFRAME_BASELINK,
                                                      message->header.stamp);
  } else if (sensor_frame_id_ == extrinsics_.GetLidarFrameId()) {
    lookup_success = extrinsics_.GetT_LIDAR_BASELINK(T_SENSORFRAME_BASELINK,
                                                     message->header.stamp);
  }

  if (!lookup_success) {
    BEAM_WARN(
        "Cannot lookup extrinsics for stamp: {}, skipping odometry message.",
        message->header.stamp.toSec());
    return;
  }

  // get pose
  Eigen::Matrix4d T_WORLD_SENSORFRAME;
  beam_common::OdometryMsgToTransformationMatrix(*message, T_WORLD_SENSORFRAME);
  Eigen::Matrix4d T_WORLD_BASELINK =
      T_WORLD_SENSORFRAME * T_SENSORFRAME_BASELINK;

  // convert and add
  geometry_msgs::TransformStamped tf_stamped;
  beam_common::EigenTransformToTransformStampedMsg(
      T_WORLD_BASELINK, message->header.stamp, message->header.seq,
      extrinsics_.GetWorldFrameId(), sensor_frame_id_, tf_stamped);
  std::string authority{"odometry"};
  poses_->setTransform(tf_stamped, authority, false);
}

}  // namespace frame_initializers
}  // namespace beam_models
