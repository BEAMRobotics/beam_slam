#include <beam_common/pose_lookup.h>

#include <beam_common/utils.h>
#include <beam_utils/log.h>

namespace beam_common {

PoseLookup::PoseLookup() {
  // get parameters from global namespace
  ros::param::get("~world_frame", world_frame_);
  ros::param::get("~baselink_frame", baselink_frame_);

  // validate parameters
  std::string error_msg{"Inputs to PoseLookup invalid."};
  if (world_frame_.empty() || baselink_frame_.empty()) {
    BEAM_ERROR(
        "Inputs to PoseLookup invalid. Parameters: world_frame and "
        "baselink_frame cannot be empty.");
    throw std::invalid_argument{error_msg};
  } else if (baselink_frame_ != extrinsics_.GetIMUFrameID() &&
             baselink_frame_ != extrinsics_.GetCameraFrameID() &&
             baselink_frame_ != extrinsics_.GetLidarFrameID()) {
    BEAM_ERROR("baselink_frame must match one of the sensor frame IDs");
    throw std::invalid_argument{error_msg};
  }
}

bool PoseLookup::CheckPoses() {
  if (poses_) {
    BEAM_ERROR("Poses cannot be empty.");
    throw std::invalid_argument{"Poses must be set for PoseLookup to function"};
    return false;
  } else {
    return true;
  }
}

bool PoseLookup::SetPoses(const std::shared_ptr<tf2::BufferCore> poses) {
  poses_ = poses;
  return CheckPoses();
}

bool PoseLookup::GetT_WORLD_SENSOR(Eigen::Matrix4d& T_WORLD_SENSOR,
                                   const std::string& sensor_frame,
                                   const ros::Time& time) {
  if (!CheckPoses()) return false;

  // get extrinsics
  Eigen::Matrix4d T_BASELINK_SENSOR;
  if (!GetT_BASELINK_SENSOR(T_BASELINK_SENSOR, sensor_frame, time)) {
    return false;
  }

  // get pose
  Eigen::Matrix4d T_WORLD_BASELINK;
  if (!GetT_WORLD_BASELINK(T_WORLD_BASELINK, time)) {
    return false;
  }

  T_WORLD_SENSOR = T_WORLD_BASELINK * T_BASELINK_SENSOR;
  return true;
}

bool PoseLookup::ThrowFrameIDError() {
  BEAM_ERROR(
      "Sensor frame ID does not match specified frames for imu, camera, or "
      "lidar");
  throw std::invalid_argument{"Invalid sensor frame ID"};
  return false;
}

bool PoseLookup::GetT_BASELINK_SENSOR(Eigen::Matrix4d& T_BASELINK_SENSOR,
                                      const std::string& sensor_frame,
                                      const ros::Time& time) {
  if (sensor_frame.empty() || baselink_frame_ == sensor_frame) {
    T_BASELINK_SENSOR.setIdentity();
    return true;
  }

  if (!extrinsics_.IsSet()) {
    BEAM_ERROR("Extrinsics between frames have not been set.");
    throw std::invalid_argument{
        "Extrinsics between imu, camera, and lidar must exist"};
    return false;
  }

  // get transform from ExtrinsicsLookup
  if (baselink_frame_ == extrinsics_.GetIMUFrameID()) {
    if (sensor_frame == extrinsics_.GetCameraFrameID()) {
      extrinsics_.GetT_IMU_CAMERA(T_BASELINK_SENSOR, time);
    } else if (sensor_frame == extrinsics_.GetLidarFrameID()) {
      extrinsics_.GetT_IMU_LIDAR(T_BASELINK_SENSOR, time);
    } else {
      return ThrowFrameIDError();
    }
  } else if (baselink_frame_ == extrinsics_.GetCameraFrameID()) {
    if (sensor_frame == extrinsics_.GetIMUFrameID()) {
      extrinsics_.GetT_CAMERA_IMU(T_BASELINK_SENSOR, time);
    } else if (sensor_frame == extrinsics_.GetLidarFrameID()) {
      extrinsics_.GetT_CAMERA_LIDAR(T_BASELINK_SENSOR, time);
    } else {
      return ThrowFrameIDError();
    }
  } else if (baselink_frame_ == extrinsics_.GetLidarFrameID()) {
    if (sensor_frame == extrinsics_.GetIMUFrameID()) {
      extrinsics_.GetT_LIDAR_IMU(T_BASELINK_SENSOR, time);
    } else if (sensor_frame == extrinsics_.GetCameraFrameID()) {
      extrinsics_.GetT_LIDAR_CAMERA(T_BASELINK_SENSOR, time);
    } else {
      return ThrowFrameIDError();
    }
  } else {
    // this condition should never be reached as we ensure that baselink is
    // set properly upon class instantiation
    return false;
  }

  return true;
}

bool PoseLookup::GetT_WORLD_BASELINK(Eigen::Matrix4d& T_WORLD_BASELINK,
                                     const ros::Time& time) {
  if (!CheckPoses()) return false;

  std::string error_msg;
  bool can_transform =
      poses_->canTransform(world_frame_, baselink_frame_, time, &error_msg);

  if (!can_transform) {
    BEAM_ERROR("Cannot lookup T_WORLD_BASELINK from poses: {}", error_msg);
    return false;
  }

  geometry_msgs::TransformStamped TROS_WORLD_BASELINK =
      poses_->lookupTransform(world_frame_, baselink_frame_, time);

  GeometryTransformStampedToEigenTransform(TROS_WORLD_BASELINK,
                                           T_WORLD_BASELINK);

  return true;
}

}  // namespace beam_common