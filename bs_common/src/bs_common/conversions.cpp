#include <bs_common/conversions.h>

#include <beam_utils/se3.h>

namespace bs_common {

Eigen::Matrix<double, 7, 1>
    TransformMatrixToVectorWithQuaternion(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  Eigen::Matrix<double, 7, 1> v;
  v << T(0, 3), T(1, 3), T(2, 3), q.w(), q.x(), q.y(), q.z();
  return v;
}

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
  T_WORLD_SENSOR = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q(o.w(), o.x(), o.y(), o.z());
  T_WORLD_SENSOR.block(0, 3, 3, 1) = Eigen::Vector3d{p.x(), p.y(), p.z()};
  T_WORLD_SENSOR.block(0, 0, 3, 3) = q.toRotationMatrix();
}

Eigen::Matrix4d
    FusePoseToEigenTransform(const fuse_variables::Position3DStamped& p,
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

void FusePoseToEigenTransform(const bs_variables::Position3D& p,
                              const bs_variables::Orientation3D& o,
                              Eigen::Matrix4d& T_WORLD_SENSOR) {
  T_WORLD_SENSOR = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q(o.w(), o.x(), o.y(), o.z());
  T_WORLD_SENSOR.block(0, 3, 3, 1) = Eigen::Vector3d{p.x(), p.y(), p.z()};
  T_WORLD_SENSOR.block(0, 0, 3, 3) = q.toRotationMatrix();
}

Eigen::Matrix4d FusePoseToEigenTransform(const bs_variables::Position3D& p,
                                         const bs_variables::Orientation3D& o) {
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

void PoseMsgToTransformationMatrix(const geometry_msgs::PoseStamped& pose,
                                   Eigen::Matrix4d& T_WORLD_SENSOR) {
  Eigen::Vector3d position;
  position[0] = pose.pose.position.x;
  position[1] = pose.pose.position.y;
  position[2] = pose.pose.position.z;
  Eigen::Quaterniond orientation;
  orientation.w() = pose.pose.orientation.w;
  orientation.x() = pose.pose.orientation.x;
  orientation.y() = pose.pose.orientation.y;
  orientation.z() = pose.pose.orientation.z;
  beam::QuaternionAndTranslationToTransformMatrix(orientation, position,
                                                  T_WORLD_SENSOR);
}

void OdometryMsgToTransformationMatrix(const nav_msgs::Odometry& odom,
                                       Eigen::Matrix4d& T_WORLD_SENSOR) {
  T_WORLD_SENSOR = Eigen::Matrix4d::Identity();
  T_WORLD_SENSOR(0, 3) = odom.pose.pose.position.x;
  T_WORLD_SENSOR(1, 3) = odom.pose.pose.position.y;
  T_WORLD_SENSOR(2, 3) = odom.pose.pose.position.z;
  Eigen::Quaterniond q;
  q.x() = odom.pose.pose.orientation.x;
  q.y() = odom.pose.pose.orientation.y;
  q.z() = odom.pose.pose.orientation.z;
  q.w() = odom.pose.pose.orientation.w;
  Eigen::Matrix3d R = q.toRotationMatrix();
  T_WORLD_SENSOR.block(0, 0, 3, 3) = R;
}

nav_msgs::Odometry TransformToOdometryMessage(
    const ros::Time& stamp, const int seq, const std::string& parent_frame_id,
    const std::string& child_frame_id, const Eigen::Matrix4d T_PARENT_CHILD,
    const Eigen::Matrix<double, 6, 6> covariance) {
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.seq = seq;
  odom_msg.header.frame_id = parent_frame_id;
  odom_msg.child_frame_id = child_frame_id;
  // handle position
  geometry_msgs::Point position;
  position.x = T_PARENT_CHILD(0, 3);
  position.y = T_PARENT_CHILD(1, 3);
  position.z = T_PARENT_CHILD(2, 3);
  odom_msg.pose.pose.position = position;
  // handle orientation
  Eigen::Matrix3d R = T_PARENT_CHILD.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  geometry_msgs::Quaternion orientation;
  orientation.x = q.x();
  orientation.y = q.y();
  orientation.z = q.z();
  orientation.w = q.w();
  odom_msg.pose.pose.orientation = orientation;
  // handle covariance
  std::vector<double> cov(covariance.data(),
                          covariance.data() +
                              covariance.rows() * covariance.cols());
  for (int i = 0; i < cov.size(); i++) { odom_msg.pose.covariance[i] = cov[i]; }

  return odom_msg;
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

void EigenTransformToOdometryMsg(const Eigen::Matrix4d& T,
                                 const ros::Time& stamp, int seq,
                                 const std::string& parent_frame_id,
                                 const std::string& child_frame_id,
                                 nav_msgs::Odometry& odom_msg) {
  odom_msg.header.seq = seq;
  odom_msg.header.frame_id = parent_frame_id;
  odom_msg.header.stamp = stamp;
  odom_msg.child_frame_id = child_frame_id;
  odom_msg.pose.pose.position.x = T(0, 3);
  odom_msg.pose.pose.position.y = T(1, 3);
  odom_msg.pose.pose.position.z = T(2, 3);
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();
}

void EigenTransformToPoseStamped(const Eigen::Matrix4d& T,
                                 const ros::Time& stamp, int seq,
                                 const std::string& frame_id,
                                 geometry_msgs::PoseStamped& pose_stamped) {
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.seq = seq;
  header.stamp = stamp;

  geometry_msgs::Point position;
  position.x = T(0, 3);
  position.y = T(1, 3);
  position.z = T(2, 3);

  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);

  geometry_msgs::Quaternion orientation;
  orientation.x = q.x();
  orientation.y = q.y();
  orientation.z = q.z();
  orientation.w = q.w();

  pose_stamped.header = header;
  pose_stamped.pose.position = position;
  pose_stamped.pose.orientation = orientation;
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

void TransformationMatrixToPoseMsg(const Eigen::Matrix4d& T_WORLD_SENSOR,
                                   const ros::Time& stamp,
                                   geometry_msgs::PoseStamped& pose) {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_SENSOR, orientation,
                                                  position);
  pose.header.stamp = stamp;
  pose.pose.position.x = position[0];
  pose.pose.position.y = position[1];
  pose.pose.position.z = position[2];
  pose.pose.orientation.w = orientation.w();
  pose.pose.orientation.x = orientation.x();
  pose.pose.orientation.y = orientation.y();
  pose.pose.orientation.z = orientation.z();
}

Eigen::Quaterniond OrientationVariableToEigenQuaternion(
    const fuse_variables::Orientation3DStamped& orientation) {
  Eigen::Quaterniond q(orientation.w(), orientation.x(), orientation.y(),
                       orientation.z());
  return q;
}

fuse_core::Vector7d ComputeDelta(const Eigen::Matrix4d& T_A_B) {
  // compute delta between previous kf and this frame
  Eigen::Vector3d t_A_B = T_A_B.block<3, 1>(0, 3);
  Eigen::Quaterniond q_A_B(T_A_B.block<3, 3>(0, 0));
  fuse_core::Vector7d delta_A_B;
  delta_A_B << t_A_B.x(), t_A_B.y(), t_A_B.z(), q_A_B.w(), q_A_B.x(), q_A_B.y(),
      q_A_B.z();
  return delta_A_B;
}

} // namespace bs_common
