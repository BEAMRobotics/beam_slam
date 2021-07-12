#pragma once

#include <beam_utils/math.h>

#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <nav_msgs/Path.h>

namespace beam_common {

inline void EigenTransformToFusePose(const Eigen::Matrix4d& T_WORLD_SENSOR,
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

inline void
    FusePoseToEigenTransform(const fuse_variables::Position3DStamped& p,
                             const fuse_variables::Orientation3DStamped& o,
                             Eigen::Matrix4d& T_WORLD_SENSOR) {
  Eigen::Quaterniond q(o.w(), o.x(), o.y(), o.z());
  T_WORLD_SENSOR.block(0, 3, 3, 1) = Eigen::Vector3d{p.x(), p.y(), p.z()};
  T_WORLD_SENSOR.block(0, 0, 3, 3) = q.toRotationMatrix();
}

inline Eigen::Matrix4d
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

inline void PoseMsgToTransformationMatrix(const geometry_msgs::PoseStamped& pose,
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

inline void InterpolateTransformFromPath(const nav_msgs::Path& path, const ros::Time& time,
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

} // namespace beam_common
