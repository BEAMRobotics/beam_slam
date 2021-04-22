#pragma once

#include <ceres/rotation.h>
#include <fuse_constraints/normal_delta_orientation_3d_cost_functor.h>
#include <fuse_core/eigen.h>
#include <fuse_core/macros.h>
#include <fuse_core/util.h>

namespace beam_constraints { namespace frame_to_frame {

/**
 * @brief Implements a cost function that models a difference between 3D
 * imu states.
 *
 * This cost function computes the difference using standard 3D transformation
 * math:
 *
 *   cost(x) = || A * [ q1^-1 * (p2 - p1) - b(0:2)        ] ||^2
 *             ||     [ q1^-1 * (v2 - v1) - b(3:5)        ] ||
 *             ||     [ AngleAxis(b(6:9)^-1 * q1^-1 * q2) ] ||
 *
 * where p1 and p2 are the position variables, v1 and v2 are the linear velocity
 * variables, q1 and q2 are the quaternion orientation variables, and the matrix
 * A and the vector b are fixed.
 *
 * Note that the cost function's quaternion components are only concerned with
 * the imaginary components (qx, qy, qz).
 */
class NormalDeltaImuState3DCostFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  NormalDeltaImuState3DCostFunctor(const fuse_core::Matrix9d& A,
                                   const Eigen::Matrix<double, 10, 1>& b);

  template <typename T>
  bool operator()(const T* const position1, const T* const velocity1,
                  const T* const orientation1, const T* const position2,
                  const T* const velocity2, const T* const orientation2,
                  T* residual) const;

private:
  fuse_core::Matrix9d A_;
  Eigen::Matrix<double, 10, 1> b_;

  fuse_constraints::NormalDeltaOrientation3DCostFunctor orientation_functor_;
};

NormalDeltaImuState3DCostFunctor::NormalDeltaImuState3DCostFunctor(
    const fuse_core::Matrix9d& A, const Eigen::Matrix<double, 10, 1>& b)
    : A_(A),
      b_(b),
      orientation_functor_(fuse_core::Matrix3d::Identity(), b_.tail<4>()) {}

template <typename T>
bool NormalDeltaImuState3DCostFunctor::operator()(
    const T* const position1, const T* const velocity1,
    const T* const orientation1, const T* const position2,
    const T* const velocity2, const T* const orientation2, T* residual) const {
  // Compute the position delta between pose1 and pose2
  T orientation1_inverse[4] = {orientation1[0], -orientation1[1],
                               -orientation1[2], -orientation1[3]};
  T position_delta[3] = {position2[0] - position1[0],
                         position2[1] - position1[1],
                         position2[2] - position1[2]};
  T velocity_delta[3] = {velocity2[0] - velocity1[0],
                         velocity2[1] - velocity1[1],
                         velocity2[2] - velocity1[2]};
  T position_delta_rotated[3];
  T velocity_delta_rotated[3];
  ceres::QuaternionRotatePoint(orientation1_inverse, position_delta,
                               position_delta_rotated);
  ceres::QuaternionRotatePoint(orientation1_inverse, velocity_delta,
                               velocity_delta_rotated);

  // Compute the first three residual terms as (position_delta - b)
  residual[0] = position_delta_rotated[0] - T(b_[0]);
  residual[1] = position_delta_rotated[1] - T(b_[1]);
  residual[2] = position_delta_rotated[2] - T(b_[2]);

  // Compute the next three residual terms as (velocity_delta - b)
  residual[3] = velocity_delta_rotated[0] - T(b_[3]);
  residual[4] = velocity_delta_rotated[1] - T(b_[4]);
  residual[5] = velocity_delta_rotated[2] - T(b_[5]);

  // Use the 3D orientation cost functor to compute the orientation delta
  orientation_functor_(orientation1, orientation2, &residual[6]);

  // Map it to Eigen, and weight it
  Eigen::Map<Eigen::Matrix<T, 9, 1>> residual_map(residual);
  residual_map.applyOnTheLeft(A_.template cast<T>());

  return true;
}

}}  // namespace namespace beam_constraints::frame_to_frame

