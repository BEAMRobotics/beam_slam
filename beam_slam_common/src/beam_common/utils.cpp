#include <beam_common/utils.h>

namespace beam_common {

void EigenTransformToFusePose(const Eigen::Matrix4d& T_WORLD_SENSOR,
                              fuse_variables::Position3DStamped& p,
                              fuse_variables::Orientation3DStamped& o) {
  // get position
  p.x() = T_WORLD_SENSOR(0, 3);
  p.y() = T_WORLD_SENSOR(1, 3);
  p.z() = T_WORLD_SENSOR(2, 3);

  // get rotation
  Eigen::Matrix3d R = T_WORLD_SENSOR.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  o.x() = q.x();
  o.y() = q.y();
  o.z() = q.z();
  o.w() = q.w();
}

void FusePoseToEigenTransform(const fuse_variables::Position3DStamped& p,
                              const fuse_variables::Orientation3DStamped& o,
                              Eigen::Matrix4d& T_WORLD_SENSOR) {
  Eigen::Quaterniond q(o.w(), o.x(), o.y(), o.z());
  T_WORLD_SENSOR.block(0, 3, 3, 1) = Eigen::Vector3d{p.x(), p.y(), p.z()};
  T_WORLD_SENSOR.block(0, 0, 3, 3) = q.toRotationMatrix();
}

Eigen::Matrix4d FusePoseToEigenTransform(
    const fuse_variables::Position3DStamped& p,
    const fuse_variables::Orientation3DStamped& o) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  // add position
  T(0, 3) = p.x();
  T(1, 3) = p.y();
  T(2, 3) = p.z();

  // add rotation
  Eigen::Quaterniond q(o.w(), o.x(), o.y(), o.z());
  Eigen::Matrix3d R = q.toRotationMatrix();
  T.block(0, 0, 3, 3) = R;
  return T;
}
  
double CalculateTrajectoryLength(
    const std::list<beam_common::ScanPose>& keyframes) {
  double length{0};
  auto iter = keyframes.begin();
  Eigen::Vector3d prev_position = iter->T_REFFRAME_CLOUD().block(0, 3, 3, 1);
  iter++;

  while (iter != keyframes.end()) {
    Eigen::Vector3d current_position =
        iter->T_REFFRAME_CLOUD().block(0, 3, 3, 1);
    Eigen::Vector3d current_motion = current_position - prev_position;
    length += current_motion.norm();
    prev_position = current_position;
    iter++;
  }

  return length;
}

void ROSStampedTransformToEigenTransform(const tf::StampedTransform& TROS,
                                         Eigen::Matrix4d& T) {
  Eigen::Matrix4f T_float{Eigen::Matrix4f::Identity()};
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
}

void TransformStampedMsgToEigenTransform(
    const geometry_msgs::TransformStamped& TROS, Eigen::Matrix4d& T) {
  Eigen::Matrix4f T_float{Eigen::Matrix4f::Identity()};
  T_float(0, 3) = TROS.transform.translation.x;
  T_float(1, 3) = TROS.transform.translation.y;
  T_float(2, 3) = TROS.transform.translation.z;
  Eigen::Quaternionf q;
  q.x() = TROS.transform.rotation.x;
  q.y() = TROS.transform.rotation.y;
  q.z() = TROS.transform.rotation.z;
  q.w() = TROS.transform.rotation.w;
  T_float.block(0, 0, 3, 3) = q.toRotationMatrix();
  T = T_float.cast<double>();
}

void EigenTransformToTransformStampedMsg(
    const Eigen::Matrix4d& T, const ros::Time& stamp, int seq,
    const std::string& parent_frame_id, const std::string& child_frame_id,
    geometry_msgs::TransformStamped& tf_stamped) {
  tf_stamped.header.stamp = stamp;
  tf_stamped.header.seq = seq;
  tf_stamped.header.frame_id = parent_frame_id;
  tf_stamped.child_frame_id = child_frame_id;
  tf_stamped.transform.translation.x = T(0, 3);
  tf_stamped.transform.translation.y = T(1, 3);
  tf_stamped.transform.translation.z = T(2, 3);
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  tf_stamped.transform.rotation.x = q.x();
  tf_stamped.transform.rotation.y = q.y();
  tf_stamped.transform.rotation.z = q.z();
  tf_stamped.transform.rotation.w = q.w();
}

void OdometryMsgToTransformedStamped(
    const nav_msgs::Odometry& message, const ros::Time& stamp, int seq,
    const std::string& parent_frame_id, const std::string& child_frame_id,
    geometry_msgs::TransformStamped& tf_stamped) {
  tf_stamped.header.stamp = stamp;
  tf_stamped.header.seq = seq;
  tf_stamped.header.frame_id = parent_frame_id;
  tf_stamped.child_frame_id = child_frame_id;
  tf_stamped.transform.translation.x = message.pose.pose.position.x;
  tf_stamped.transform.translation.y = message.pose.pose.position.y;
  tf_stamped.transform.translation.z = message.pose.pose.position.z;
  tf_stamped.transform.rotation = message.pose.pose.orientation;
}

}  // namespace beam_common
