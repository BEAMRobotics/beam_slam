#include <beam_common/extrinsics_lookup.h>

#include <beam_utils/log.h>
#include <beam_utils/math.h>

namespace beam_common {

ExtrinsicsLookup::ExtrinsicsLookup(const Params& params) : params(params) {
  if (params.imu_frame.empty() || params.camera_frame.empty() ||
      params.lidar_frame.empty()) {
    BEAM_ERROR(
        "Inputs to ExtrinsicsLookup invalid. You must supply a frame name for "
        "each of the 3 sensor types: imu, camera, lidar");
    throw std::invalid_argument{"Inputs to ExtrinsicsLookup invalid."};
  }
}

bool ExtrinsicsLookup::GetT_CAMERA_IMU(Eigen::Matrix4d& T,
                                       const ros::Time& time) {
  // check if already known
  if(params.static_extrinsics && T_IMU_CAMERA_set_){
    T = beam::InvertTransform(T_IMU_CAMERA_);
    return true
  }
  
  // get extrinsics
  if (!GetTransform(T, params.camera_frame, params.imu_frame, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if(params.static_extrinsics){
    T_IMU_CAMERA_ = beam::InvertTransform(T);
    T_IMU_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_IMU_CAMERA(Eigen::Matrix4d& T,
                                       const ros::Time& time) {
  // check if already known
  if(params.static_extrinsics && T_IMU_CAMERA_set_){
    T = T_IMU_CAMERA_;
    return true
  }
  
  // get extrinsics
  if (!GetTransform(T, params.imu_frame, params.camera_frame, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if(params.static_extrinsics){
    T_IMU_CAMERA_ = T;
    T_IMU_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_CAMERA_LIDAR(Eigen::Matrix4d& T,
                                       const ros::Time& time) {
  // check if already known
  if(params.static_extrinsics && T_LIDAR_CAMERA_set_){
    T = beam::InvertTransform(T_LIDAR_CAMERA_);
    return true
  }
  
  // get extrinsics
  if (!GetTransform(T, params.camera_frame, params.lidar_frame, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if(params.static_extrinsics){
    T_LIDAR_CAMERA_ = beam::InvertTransform(T);
    T_LIDAR_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_LIDAR_CAMERA(Eigen::Matrix4d& T,
                                       const ros::Time& time) {
  // check if already known
  if(params.static_extrinsics && T_LIDAR_CAMERA_set_){
    T = T_LIDAR_CAMERA_;
    return true
  }
  
  // get extrinsics
  if (!GetTransform(T, params.lidar_frame, params.camera_frame, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if(params.static_extrinsics){
    T_LIDAR_CAMERA_ = T;
    T_LIDAR_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_IMU_LIDAR(Eigen::Matrix4d& T,
                                       const ros::Time& time) {
  // check if already known
  if(params.static_extrinsics && T_LIDAR_IMU_set_){
    T = beam::InvertTransform(T_LIDAR_IMU_);
    return true
  }
  
  // get extrinsics
  if (!GetTransform(T, params.imu_frame, params.lidar_frame, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if(params.static_extrinsics){
    T_LIDAR_IMU_ = beam::InvertTransform(T);
    T_LIDAR_IMU_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_LIDAR_IMU(Eigen::Matrix4d& T,
                                       const ros::Time& time) {
  // check if already known
  if(params.static_extrinsics && T_LIDAR_IMU_set_){
    T = T_LIDAR_IMU_;
    return true
  }
  
  // get extrinsics
  if (!GetTransform(T, params.lidar_frame, params.imu_frame, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if(params.static_extrinsics){
    T_LIDAR_IMU_ = T;
    T_LIDAR_IMU_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetTransform(Eigen::Matrix4d& T, const std::string& to_frame,
                              const std::string& from_frame,
                              const ros::Time& time) {
  tf::StampedTransform TROS;
  try {
    tf_listener_.lookupTransform(to_frame, from_frame,
                                 time, TROS);
  } catch (tf::TransformException& ex) {
    if (params.static_extrinsics) {
      BEAM_WARN("Cannot lookup static extrinsics between frames: {} , {}",
                to_frame, from_frame);
    } else {
      BEAM_WARN("Cannot lookup dynamic extrinsics for t = %.10f",
                time.toSec());
    }
    return false;
  }

  // convert to Eigen matrix
  Eigen::Matrix4f T_float = Eigen::Matrix4f::Identity();
  T_float(0, 3) = TROS_BASELINK_SENSOR.getOrigin().getX();
  T_float(1, 3) = TROS_BASELINK_SENSOR.getOrigin().getY();
  T_float(2, 3) = TROS_BASELINK_SENSOR.getOrigin().getZ();
  Eigen::Quaternionf q;
  q.x() = TROS_BASELINK_SENSOR.getRotation().getX();
  q.y() = TROS_BASELINK_SENSOR.getRotation().getY();
  q.z() = TROS_BASELINK_SENSOR.getRotation().getZ();
  q.w() = TROS_BASELINK_SENSOR.getRotation().getW();
  T_float.block(0, 0, 3, 3) = q.toRotationMatrix();
  T = T.cast<double>();

  return true;
}

}  // namespace beam_common