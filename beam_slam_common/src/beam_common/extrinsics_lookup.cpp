#include <beam_common/extrinsics_lookup.h>

#include <beam_utils/log.h>
#include <beam_utils/math.h>

namespace beam_common {

ExtrinsicsLookup& ExtrinsicsLookup::GetInstance() {
  static ExtrinsicsLookup instance_;
  return instance_;
}

ExtrinsicsLookup::ExtrinsicsLookup() {
  // get parameters from global namespace
  ros::param::get("~imu_frame_", imu_frame_);
  ros::param::get("~camera_frame_", camera_frame_);
  ros::param::get("~lidar_frame_", lidar_frame_);
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
  if (static_extrinsics && T_IMU_CAMERA_set_) {
    T = beam::InvertTransform(T_IMU_CAMERA_);
    return true
  }

  // get extrinsics
  if (!GetTransform(T, camera_frame_, imu_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics) {
    T_IMU_CAMERA_ = beam::InvertTransform(T);
    T_IMU_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_IMU_CAMERA(Eigen::Matrix4d& T,
                                       const ros::Time& time) {
  // check if already known
  if (static_extrinsics && T_IMU_CAMERA_set_) {
    T = T_IMU_CAMERA_;
    return true
  }

  // get extrinsics
  if (!GetTransform(T, imu_frame_, camera_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics) {
    T_IMU_CAMERA_ = T;
    T_IMU_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_CAMERA_LIDAR(Eigen::Matrix4d& T,
                                         const ros::Time& time) {
  // check if already known
  if (static_extrinsics && T_LIDAR_CAMERA_set_) {
    T = beam::InvertTransform(T_LIDAR_CAMERA_);
    return true
  }

  // get extrinsics
  if (!GetTransform(T, camera_frame_, lidar_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics) {
    T_LIDAR_CAMERA_ = beam::InvertTransform(T);
    T_LIDAR_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_LIDAR_CAMERA(Eigen::Matrix4d& T,
                                         const ros::Time& time) {
  // check if already known
  if (static_extrinsics && T_LIDAR_CAMERA_set_) {
    T = T_LIDAR_CAMERA_;
    return true
  }

  // get extrinsics
  if (!GetTransform(T, lidar_frame_, camera_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics) {
    T_LIDAR_CAMERA_ = T;
    T_LIDAR_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_IMU_LIDAR(Eigen::Matrix4d& T,
                                      const ros::Time& time) {
  // check if already known
  if (static_extrinsics && T_LIDAR_IMU_set_) {
    T = beam::InvertTransform(T_LIDAR_IMU_);
    return true
  }

  // get extrinsics
  if (!GetTransform(T, imu_frame_, lidar_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics) {
    T_LIDAR_IMU_ = beam::InvertTransform(T);
    T_LIDAR_IMU_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_LIDAR_IMU(Eigen::Matrix4d& T,
                                      const ros::Time& time) {
  // check if already known
  if (static_extrinsics && T_LIDAR_IMU_set_) {
    T = T_LIDAR_IMU_;
    return true
  }

  // get extrinsics
  if (!GetTransform(T, lidar_frame_, imu_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics) {
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
    if (static_extrinsics) {
      BEAM_WARN("Cannot lookup static extrinsics between frames: {} , {}",
                to_frame, from_frame);
    } else {
      BEAM_WARN("Cannot lookup dynamic extrinsics for t = %.10f", time.toSec());
    }
    return false;
  }

  // convert to Eigen matrix
  Eigen::Matrix4f T_float = Eigen::Matrix4f::Identity();
  T_float(0, 3) = TROS.getOrigin().getX();
  T_float(1, 3) = TROS.getOrigin().getY();
  T_float(2, 3) = TROS.getOrigin().getZ();
  Eigen::Quaternionf q;
  q.x() = TROS.getRotation().getX();
  q.y() = TROS.getRotation().getY();
  q.z() = TROS.getRotation().getZ();
  q.w() = TROS.getRotation().getW();
  T_float.block(0, 0, 3, 3) = q.toRotationMatrix();
  T = T_float.cast<double>();

  return true;
}

}  // namespace beam_common