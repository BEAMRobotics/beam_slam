#include <bs_common/pose_lookup.h>

#include <beam_utils/log.h>
#include <bs_common/utils.h>

namespace bs_common {

PoseLookup::PoseLookup(const std::shared_ptr<tf2::BufferCore> poses)
    : poses_(poses) {}

bool PoseLookup::GetT_WORLD_SENSOR(Eigen::Matrix4d& T_WORLD_SENSOR,
                                   const std::string& sensor_frame,
                                   const ros::Time& time,
                                   std::string& error_msg) {
  Eigen::Matrix4d T_BASELINK_SENSOR;
  if (!extrinsics_.GetT_BASELINK_SENSOR(T_BASELINK_SENSOR, sensor_frame,
                                        time)) {
    return false;
  }

  Eigen::Matrix4d T_WORLD_BASELINK;
  if (!GetT_WORLD_BASELINK(T_WORLD_BASELINK, time, error_msg)) { return false; }

  T_WORLD_SENSOR = T_WORLD_BASELINK * T_BASELINK_SENSOR;
  return true;
}

bool PoseLookup::GetT_WORLD_BASELINK(Eigen::Matrix4d& T_WORLD_BASELINK,
                                     const ros::Time& time,
                                     std::string& error_msg) {
  bool can_transform =
      poses_->canTransform(extrinsics_.GetWorldFrameId(),
                           extrinsics_.GetBaselinkFrameId(), time, &error_msg);

  if (!can_transform) { return false; }

  geometry_msgs::TransformStamped TROS_WORLD_BASELINK = poses_->lookupTransform(
      extrinsics_.GetWorldFrameId(), extrinsics_.GetBaselinkFrameId(), time);

  TransformStampedMsgToEigenTransform(TROS_WORLD_BASELINK, T_WORLD_BASELINK);

  return true;
}

} // namespace bs_common