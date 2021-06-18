#include <beam_common/extrinsics_lookup.h>
#include <beam_common/utils.h>

#include <beam_utils/log.h>
#include <beam_utils/math.h>

namespace beam_common {

ExtrinsicsLookup& ExtrinsicsLookup::GetInstance() {
  static ExtrinsicsLookup instance;
  return instance;
}

ExtrinsicsLookup::ExtrinsicsLookup() {
  // get parameters from global namespace
  ros::param::get("~imu_frame", imu_frame_);
  ros::param::get("~camera_frame", camera_frame_);
  ros::param::get("~lidar_frame", lidar_frame_);
  ros::param::get("~static_extrinsics", static_extrinsics_);

  // validate parameters
  if (imu_frame_.empty() || camera_frame_.empty() || lidar_frame_.empty()) {
    BEAM_ERROR(
        "Inputs to ExtrinsicsLookup invalid. You must supply a frame name "
        "for each of the 3 sensor types: imu, camera, lidar");
    throw std::invalid_argument{"Inputs to ExtrinsicsLookup invalid."};
  }
}

bool ExtrinsicsLookup::GetT_CAMERA_IMU(Eigen::Matrix4d& T,
                                       const ros::Time& time) {
  // check if already known
  if (static_extrinsics_ && T_IMU_CAMERA_set_) {
    T = beam::InvertTransform(T_IMU_CAMERA_);
    return true;
  }

  // get extrinsics
  if (!GetTransform(T, camera_frame_, imu_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics_) {
    T_IMU_CAMERA_ = beam::InvertTransform(T);
    T_IMU_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_IMU_CAMERA(Eigen::Matrix4d& T,
                                       const ros::Time& time) {
  // check if already known
  if (static_extrinsics_ && T_IMU_CAMERA_set_) {
    T = T_IMU_CAMERA_;
    return true;
  }

  // get extrinsics
  if (!GetTransform(T, imu_frame_, camera_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics_) {
    T_IMU_CAMERA_ = T;
    T_IMU_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_CAMERA_LIDAR(Eigen::Matrix4d& T,
                                         const ros::Time& time) {
  // check if already known
  if (static_extrinsics_ && T_LIDAR_CAMERA_set_) {
    T = beam::InvertTransform(T_LIDAR_CAMERA_);
    return true;
  }

  // get extrinsics
  if (!GetTransform(T, camera_frame_, lidar_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics_) {
    T_LIDAR_CAMERA_ = beam::InvertTransform(T);
    T_LIDAR_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_LIDAR_CAMERA(Eigen::Matrix4d& T,
                                         const ros::Time& time) {
  // check if already known
  if (static_extrinsics_ && T_LIDAR_CAMERA_set_) {
    T = T_LIDAR_CAMERA_;
    return true;
  }

  // get extrinsics
  if (!GetTransform(T, lidar_frame_, camera_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics_) {
    T_LIDAR_CAMERA_ = T;
    T_LIDAR_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_IMU_LIDAR(Eigen::Matrix4d& T,
                                      const ros::Time& time) {
  // check if already known
  if (static_extrinsics_ && T_LIDAR_IMU_set_) {
    T = beam::InvertTransform(T_LIDAR_IMU_);
    return true;
  }

  // get extrinsics
  if (!GetTransform(T, imu_frame_, lidar_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics_) {
    T_LIDAR_IMU_ = beam::InvertTransform(T);
    T_LIDAR_IMU_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_LIDAR_IMU(Eigen::Matrix4d& T,
                                      const ros::Time& time) {
  // check if already known
  if (static_extrinsics_ && T_LIDAR_IMU_set_) {
    T = T_LIDAR_IMU_;
    return true;
  }

  // get extrinsics
  if (!GetTransform(T, lidar_frame_, imu_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics_) {
    T_LIDAR_IMU_ = T;
    T_LIDAR_IMU_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetTransform(Eigen::Matrix4d& T,
                                    const std::string& to_frame,
                                    const std::string& from_frame,
                                    const ros::Time& time) {
  tf::StampedTransform TROS;
  try {
    tf_listener_.lookupTransform(to_frame, from_frame, time, TROS);
  } catch (tf::TransformException& ex) {
    if (static_extrinsics_) {
      BEAM_WARN("Cannot lookup static extrinsics between frames: {} , {}",
                to_frame, from_frame);
    } else {
      BEAM_WARN("Cannot lookup dynamic extrinsics for t = %.10f", time.toSec());
    }
    return false;
  }

  ROSStampedTransformToEigenTransform(TROS, T);

  return true;
}

}  // namespace beam_common