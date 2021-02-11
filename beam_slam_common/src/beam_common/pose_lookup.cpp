#include <beam_common/pose_lookup.h>

#include <beam_utils/log.h>

namespace beam_common {

PoseLookup::PoseLookup(const Params& params) : params_(params) {
  if (params_.poses == nullptr || params_.baselink_frame.empty() ||
      params_.world_frame.empty()) {
    BEAM_ERROR("Inputs to PoseLookup invalid. Parameters: poses, "
               "baselink_frame and world_frame cannot be empty.");
    throw std::invalid_argument{"Inputs to PoseLookup invalid."};
  }
}

bool PoseLookup::GetT_WORLD_SENSOR(Eigen::Matrix4d& T_WORLD_SENSOR,
                                   const ros::Time& time) {
  // get extrinsics
  if (!GetT_BASELINK_SENSOR(T_BASELINK_SENSOR_, time)) { return false; }

  // get pose
  Eigen::Matrix4d T_WORLD_BASELINK;
  if (!GetT_WORLD_BASELINK(T_WORLD_BASELINK, time)) { return false; }

  T_WORLD_SENSOR = T_WORLD_BASELINK * T_BASELINK_SENSOR_;
  return true;
}

bool PoseLookup::GetT_BASELINK_SENSOR(Eigen::Matrix4d& T_WORLD_BASELINK,
                                      const ros::Time& time) {
  // check if extrinsics are already set
  if (params_.static_extrinsics && extrinsics_set_) { return true; }

  // check if sensor frame given, if not set extrinsics to identity
  if (params_.sensor_frame.empty()) {
    T_WORLD_BASELINK.setIdentity();
    extrinsics_set_ = true;
    return true;
  }

  ros::Time lookup_time = ros::Time(0);
  if (!params_.static_extrinsics) { lookup_time = time; }
  
  tf::StampedTransform TROS_BASELINK_SENSOR;
  try {
    tf_listener_.lookupTransform(params_.baselink_frame, params_.sensor_frame,
                                 lookup_time, TROS_BASELINK_SENSOR);
    extrinsics_set_ = true;
  } catch (tf::TransformException& ex) {
    if (params_.static_extrinsics) {
      BEAM_WARN("Cannot lookup static extrinsics.");
    } else {
      BEAM_WARN("Cannot lookup dynamic extrinsics for t = %.10f",
                lookup_time.toSec());
    }
    return false;
  }

  // convert to Eigen matrix
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0,3) = TROS_BASELINK_SENSOR.getOrigin().getX();
  T(1,3) = TROS_BASELINK_SENSOR.getOrigin().getY();
  T(2,3) = TROS_BASELINK_SENSOR.getOrigin().getZ();
  Eigen::Quaternionf q;
  q.x() = TROS_BASELINK_SENSOR.getRotation().getX();
  q.y() = TROS_BASELINK_SENSOR.getRotation().getY();
  q.z() = TROS_BASELINK_SENSOR.getRotation().getZ();
  q.w() = TROS_BASELINK_SENSOR.getRotation().getW();
  T.block(0, 0, 3, 3) = q.toRotationMatrix();
  T_WORLD_BASELINK = T.cast<double>();

  return true;
}

bool PoseLookup::GetT_WORLD_BASELINK(Eigen::Matrix4d& T_WORLD_BASELINK,
                                     const ros::Time& time) {
  std::string error_msg;
  bool can_transform = params_.poses->canTransform(
      params_.world_frame, params_.baselink_frame, time, &error_msg);

  if (!can_transform) {
    BEAM_ERROR("Cannot lookup T_WORLD_BASELINK from poses: {}", error_msg);
    return false;
  }

  geometry_msgs::TransformStamped TROS_WORLD_BASELINK =
      params_.poses->lookupTransform(params_.world_frame,
                                     params_.baselink_frame, time);

  // convert to eigen
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3) = TROS_WORLD_BASELINK.transform.translation.x;
  T(1, 3) = TROS_WORLD_BASELINK.transform.translation.y;
  T(2, 3) = TROS_WORLD_BASELINK.transform.translation.z;
  T(2, 3) = TROS_WORLD_BASELINK.transform.translation.z;
  Eigen::Quaternionf q;
  q.x() = TROS_WORLD_BASELINK.transform.rotation.x;
  q.y() = TROS_WORLD_BASELINK.transform.rotation.y;
  q.z() = TROS_WORLD_BASELINK.transform.rotation.z;
  q.w() = TROS_WORLD_BASELINK.transform.rotation.w;
  T.block(0, 0, 3, 3) = q.toRotationMatrix();
  T_WORLD_BASELINK = T.cast<double>();
  return true;
}

} // namespace beam_common