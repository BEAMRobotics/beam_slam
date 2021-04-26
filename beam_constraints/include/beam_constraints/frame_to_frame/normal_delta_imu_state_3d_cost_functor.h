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
 *   cost(x) = || A * [ AngleAxis(b(0:3)^-1 * q1^-1 * q2) ] ||^2
 *             ||     [ q1^-1 * (v2 - v1) - b(4:6)        ] ||
 *             ||     [ q1^-1 * (p2 - p1) - b(7:9)        ] ||
 *             ||     [ (ba2 - ba1) - b(10:12)            ] ||
 *             ||     [ (bg2 - bg1) - b(13:15)            ] ||
 *
 * where q1 and q2 are the quaternion orientation variables, v1 and v2 are the
 * linear velocity variables, p1 and p2 are the position variables, ba1 and ba2
 * are the acceleration bias variables, bg1 and bg2 are the gyroscope bias
 * variables, and the matrix A and the vector b are fixed.
 *
 * Note that the cost function's quaternion components are only concerned with
 * the imaginary components (qx, qy, qz).
 */
class NormalDeltaImuState3DCostFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  NormalDeltaImuState3DCostFunctor(const Eigen::Matrix<double, 15, 15>& A,
                                   const Eigen::Matrix<double, 16, 1>& b);

  template <typename T>
  bool operator()(const T* const orientation1, const T* const velocity1,
                  const T* const position1, const T* const accelbias1,
                  const T* const gyrobias1, const T* const orientation2,
                  const T* const velocity2, const T* const position2,
                  const T* const accelbias2, const T* const gyrobias2,
                  T* residual) const;

private:
  Eigen::Matrix<double, 15, 15> A_;
  Eigen::Matrix<double, 16, 1> b_;

  fuse_constraints::NormalDeltaOrientation3DCostFunctor orientation_functor_;
};

NormalDeltaImuState3DCostFunctor::NormalDeltaImuState3DCostFunctor(
    const Eigen::Matrix<double, 15, 15>& A,
    const Eigen::Matrix<double, 16, 1>& b)
    : A_(A),
      b_(b),
      orientation_functor_(fuse_core::Matrix3d::Identity(), b_.head<4>()) {}

template <typename T>
bool NormalDeltaImuState3DCostFunctor::operator()(
    const T* const orientation1, const T* const velocity1,
    const T* const position1, const T* const accelbias1,
    const T* const gyrobias1, const T* const orientation2,
    const T* const velocity2, const T* const position2,
    const T* const accelbias2, const T* const gyrobias2, T* residual) const {
  // Compute the delta between imu states
  T orientation1_inverse[4] = {orientation1[0], -orientation1[1],
                               -orientation1[2], -orientation1[3]};
  T velocity_delta[3] = {velocity2[0] - velocity1[0],
                         velocity2[1] - velocity1[1],
                         velocity2[2] - velocity1[2]};
  T position_delta[3] = {position2[0] - position1[0],
                         position2[1] - position1[1],
                         position2[2] - position1[2]};
  T bias_accel_delta[3] = {accelbias2[0] - accelbias1[0],
                           accelbias2[1] - accelbias1[1],
                           accelbias2[2] - accelbias1[2]};
  T bias_gyro_delta[3] = {gyrobias2[0] - gyrobias1[0],
                          gyrobias2[1] - gyrobias1[1],
                          gyrobias2[2] - gyrobias1[2]};

  // apply transformation to velocity and position
  T velocity_delta_rotated[3];
  T position_delta_rotated[3];
  ceres::QuaternionRotatePoint(orientation1_inverse, velocity_delta,
                               velocity_delta_rotated);
  ceres::QuaternionRotatePoint(orientation1_inverse, position_delta,
                               position_delta_rotated);

  // Use the 3D orientation cost functor to compute the orientation delta
  orientation_functor_(orientation1, orientation2, &residual[0]);

  // Compute the next three residual terms as (velocity_delta - b)
  residual[3] = velocity_delta_rotated[0] - T(b_[4]);
  residual[4] = velocity_delta_rotated[1] - T(b_[5]);
  residual[5] = velocity_delta_rotated[2] - T(b_[6]);

  // Compute the next three residual terms as (position_delta - b)
  residual[6] = position_delta_rotated[0] - T(b_[7]);
  residual[7] = position_delta_rotated[1] - T(b_[8]);
  residual[8] = position_delta_rotated[2] - T(b_[9]);

  // Compute the next three residual terms as (bias_accel_delta - b)
  residual[9] = bias_accel_delta[0] - T(b_[10]);
  residual[10] = bias_accel_delta[1] - T(b_[11]);
  residual[11] = bias_accel_delta[2] - T(b_[12]);

  // Compute the next three residual terms as (bias_gyro_delta - b)
  residual[12] = bias_gyro_delta[0] - T(b_[13]);
  residual[13] = bias_gyro_delta[1] - T(b_[14]);
  residual[14] = bias_gyro_delta[2] - T(b_[15]);

  // Map it to Eigen, and weight it
  Eigen::Map<Eigen::Matrix<T, 15, 1>> residual_map(residual);
  residual_map.applyOnTheLeft(A_.template cast<T>());

  return true;
}

}}  // namespace beam_constraints::frame_to_frame
