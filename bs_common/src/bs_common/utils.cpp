#include <bs_common/utils.h>

namespace bs_common {

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

void InterpolateTransformFromPath(const nav_msgs::Path& path,
                                  const ros::Time& time,
                                  Eigen::Matrix4d& T_WORLD_SENSOR) {
  for (int i = 0; i < path.poses.size(); i++) {
    if (time < path.poses[i + 1].header.stamp &&
        time >= path.poses[i].header.stamp) {
      Eigen::Matrix4d pose1, pose2;
      PoseMsgToTransformationMatrix(path.poses[i], pose1);
      PoseMsgToTransformationMatrix(path.poses[i + 1], pose2);
      T_WORLD_SENSOR = beam::InterpolateTransform(
          pose1, beam::RosTimeToChrono(path.poses[i].header.stamp), pose2,
          beam::RosTimeToChrono(path.poses[i + 1].header.stamp),
          beam::RosTimeToChrono(time));
    }
  }
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

void InterpolateTransformFromPath(
    const std::vector<geometry_msgs::PoseStamped>& poses, const ros::Time& time,
    Eigen::Matrix4d& T_WORLD_SENSOR) {
  for (int i = 0; i < poses.size(); i++) {
    if (time < poses[i + 1].header.stamp && time >= poses[i].header.stamp) {
      Eigen::Matrix4d pose1, pose2;
      PoseMsgToTransformationMatrix(poses[i], pose1);
      PoseMsgToTransformationMatrix(poses[i + 1], pose2);
      T_WORLD_SENSOR = beam::InterpolateTransform(
          pose1, beam::RosTimeToChrono(poses[i].header.stamp), pose2,
          beam::RosTimeToChrono(poses[i + 1].header.stamp),
          beam::RosTimeToChrono(time));
    }
  }
}

std::string GetBeamSlamConfigPath() {
  std::string current_path_from_beam_slam = "bs_common/src/bs_common/utils.cpp";
  std::string config_root_location = __FILE__;
  config_root_location.erase(
      config_root_location.end() - current_path_from_beam_slam.length(),
      config_root_location.end());
  config_root_location += "beam_slam_launch/config/";
  if (!boost::filesystem::exists(config_root_location)) {
    BEAM_ERROR("Cannot locate beam slam config folder. Expected to be at: {}",
               config_root_location);           
  }
  return config_root_location;
}

}  // namespace bs_common
