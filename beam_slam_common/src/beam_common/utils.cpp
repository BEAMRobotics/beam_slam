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

}  // namespace beam_common
