#include <beam_common/pose_lookup.h>

#include <beam_common/utils.h>
#include <beam_utils/log.h>

namespace beam_common {

PoseLookup::PoseLookup(const std::shared_ptr<tf2::BufferCore> poses)
    : poses_(poses) {}

bool PoseLookup::GetT_WORLD_SENSOR(Eigen::Matrix4d& T_WORLD_SENSOR,
                                   const std::string& sensor_frame,
                                   const ros::Time& time) {
  Eigen::Matrix4d T_BASELINK_SENSOR;
  if (!GetT_BASELINK_SENSOR(T_BASELINK_SENSOR, sensor_frame, time)) {
    return false;
  }

  Eigen::Matrix4d T_WORLD_BASELINK;
  if (!GetT_WORLD_BASELINK(T_WORLD_BASELINK, time)) {
    return false;
  }

  T_WORLD_SENSOR = T_WORLD_BASELINK * T_BASELINK_SENSOR;
  return true;
}

bool PoseLookup::GetT_WORLD_BASELINK(Eigen::Matrix4d& T_WORLD_BASELINK,
                                     const ros::Time& time) {
  std::string error_msg;
  bool can_transform =
      poses_->canTransform(extrinsics_.GetWorldFrameId(),
                           extrinsics_.GetBaselinkFrameId(), time, &error_msg);

  if (!can_transform) {
    BEAM_ERROR("Cannot lookup T_WORLD_BASELINK from poses: {}", error_msg);
    return false;
  }

  geometry_msgs::TransformStamped TROS_WORLD_BASELINK = poses_->lookupTransform(
      extrinsics_.GetWorldFrameId(), extrinsics_.GetBaselinkFrameId(), time);

  TransformStampedMsgToEigenTransform(TROS_WORLD_BASELINK, T_WORLD_BASELINK);

  return true;
}

bool PoseLookup::GetT_BASELINK_SENSOR(Eigen::Matrix4d& T_BASELINK_SENSOR,
                                      const std::string& sensor_frame,
                                      const ros::Time& time) {
  if (sensor_frame == extrinsics_.GetImuFrameId()) {
    extrinsics_.GetT_BASELINK_IMU(T_BASELINK_SENSOR, time);
  } else if (sensor_frame == extrinsics_.GetCameraFrameId()) {
    extrinsics_.GetT_BASELINK_CAMERA(T_BASELINK_SENSOR, time);
  } else if (sensor_frame == extrinsics_.GetLidarFrameId()) {
    extrinsics_.GetT_BASELINK_LIDAR(T_BASELINK_SENSOR, time);
  } else {
    BEAM_WARN(
        "Cannot lookup extrinsics between baselink frame: {} and sensor frame: "
        "{}. Ensure sensor frame ID matches either imu, camera, or lidar "
        "frames and transformation exists at time: {}",
        GetBaselinkFrameId(), sensor_frame, time.toSec());
    return false;
  }

  return true;
}

}  // namespace beam_common