#pragma once

#include <beam_utils/math.h>

#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>

namespace beam_common {

inline void EigenTransformToFusePose(const Eigen::Matrix4d& T,
                                     fuse_variables::Position3DStamped& p,
                                     fuse_variables::Orientation3DStamped& o) {
  // get position
  p.x() = T(0, 3);
  p.y() = T(1, 3);
  p.z() = T(2, 3);

  // get rotation
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  o.x() = q.x();
  o.y() = q.y();
  o.z() = q.z();
  o.w() = q.w();
}

inline void FusePoseToEigenTransform(
    const fuse_variables::Position3DStamped& p,
    const fuse_variables::Orientation3DStamped& o, Eigen::Matrix4d& T) {
  Eigen::Quaterniond q(o.w(), o.x(), o.y(), o.z());
  T.block(0, 3, 3, 1) = Eigen::Vector3d{p.x(), p.y(), p.z()};
  T.block(0, 0, 3, 3) = q.toRotationMatrix();
}

inline Eigen::Matrix4d FusePoseToEigenTransform(
    const fuse_variables::Position3DStamped& p,
    const fuse_variables::Orientation3DStamped& o) {
  Eigen::Matrix4d T{Eigen::Matrix4d::Identity()};
  FusePoseToEigenTransform(p, o, T);
  return T;
}

inline void ROSStampedTransformToEigenTransform(
    const tf::StampedTransform& TROS, Eigen::Matrix4d& T) {
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

inline void GeometryTransformStampedToEigenTransform(
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

inline void PoseMsgToTransformationMatrix(
    const geometry_msgs::PoseStamped& pose, Eigen::Matrix4d& T_WORLD_SENSOR) {
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

inline void InterpolateTransformFromPath(const nav_msgs::Path& path,
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

}  // namespace beam_common
