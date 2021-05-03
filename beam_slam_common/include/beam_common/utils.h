#pragma once

#include <fuse_core/eigen.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

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

inline void FusePoseToEigenTransform(const fuse_variables::Position3DStamped& p,
                                     const fuse_variables::Orientation3DStamped& o,
                                     Eigen::Matrix4d& T_WORLD_SENSOR) {
  Eigen::Quaterniond q(o.w(), o.x(), o.y(), o.z());
  T_WORLD_SENSOR.block(0, 3, 3, 1) = Eigen::Vector3d{p.x(), p.y(), p.z()};
  T_WORLD_SENSOR.block(0, 0, 3, 3) = q.toRotationMatrix();
}

// Temporary functions form 'slamtools', to be replaced by manif or sophus after testing... 
inline Eigen::Matrix3d hat(const Eigen::Vector3d &w) {
    return (Eigen::Matrix3d() << 0, -w.z(), w.y(),
            w.z(), 0, -w.x(),
            -w.y(), w.x(), 0)
        .finished();
}

inline Eigen::Quaterniond expmap(const Eigen::Vector3d &w) {
    Eigen::AngleAxisd aa(w.norm(), w.stableNormalized());
    Eigen::Quaterniond q;
    q = aa;
    return q;
}

inline Eigen::Vector3d logmap(const Eigen::Quaterniond &q) {
    Eigen::AngleAxisd aa(q);
    return aa.angle() * aa.axis();
}

inline Eigen::Matrix3d right_Jacobian(const Eigen::Vector3d &w) {
  static const double root2_eps = sqrt(std::numeric_limits<double>::epsilon());
  static const double root4_eps = sqrt(root2_eps);
  static const double twopi = 2.0 * 3.14159265358979323846;
  static const double qdrt720 = sqrt(sqrt(720.0));
  static const double qdrt5040 = sqrt(sqrt(5040.0));
  static const double sqrt24 = sqrt(24.0);
  static const double sqrt120 = sqrt(120.0);

  double angle = w.norm();
  double cangle = cos(angle);
  double sangle = sin(angle);
  double angle2 = angle * angle;

  double cos_term;
  // compute (1-cos(x))/x^2, its taylor expansion around 0 is 1/2-x^2/24+x^4/720+o(x^6)
  if (angle > root4_eps * qdrt720) {
      cos_term = (1 - cangle) / angle2;
  } else { // use taylor expansion to avoid singularity
      cos_term = 0.5;
      if (angle > root2_eps * sqrt24) { // we have to include x^2 term
          cos_term -= angle2 / 24.0;
      }
  }

  double sin_term;
  // compute (x-sin(x))/x^3, its taylor expansion around 0 is 1/6-x^2/120+x^4/5040+o(x^6)
  if (angle > root4_eps * qdrt5040) {
      sin_term = (angle - sangle) / (angle * angle2);
  } else {
      sin_term = 1.0 / 6.0;
      if (angle > root2_eps * sqrt120) { // we have to include x^2 term
          sin_term -= angle2 / 120.0;
      }
  }

  Eigen::Matrix3d hat_w = hat(w);
  return Eigen::Matrix3d::Identity() - cos_term * hat_w + sin_term * hat_w * hat_w;
}

} // namespace beam_common
