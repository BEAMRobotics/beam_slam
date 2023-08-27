#pragma once

#include <Eigen/Core>
#include <ceres/rotation.h>
#include <fuse_constraints/normal_prior_orientation_3d_cost_functor.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

namespace bs_constraints {

/**
 * @brief Create an absolute constraint on orientation to align with gravity.
 * This constrains 2DOF including roll and pitch, but not yaw which cannot be
 * observed from IMU alone (excluding magnetometer which we do not trust).
 *
 * The cost function is of the form:
 *
 *   cost(x) = || A * [  e_x ] ||^2
 *             ||     [  e_y ] ||
 *
 * where, the matrix A is the sqrt inv cov which weighs the residuals, and the
 * residuals are defined as follows:
 *
 * |e_x| = [ R_W_B * gravity_in_baselink ]
 * |e_y|
 *
 * Note we ignore the z term since we only care about the
 * deviation in x and y directions (roll & pitch)
 *
 */
class GravityAlignmentCostFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  GravityAlignmentCostFunctor(
      const Eigen::Matrix<double, 2, 2>& A,
      const Eigen::Matrix<double, 3, 1>& gravity_in_baselink);

  template <typename T>
  bool operator()(const T* const qwxyz_World_Baselink, T* residual) const;

private:
  Eigen::Matrix<double, 2, 2> A_;
  Eigen::Matrix<double, 3, 1> gravity_in_baselink_;
};

GravityAlignmentCostFunctor::GravityAlignmentCostFunctor(
    const Eigen::Matrix<double, 2, 2>& A,
    const Eigen::Matrix<double, 3, 1>& gravity_in_baselink)
    : A_(A), gravity_in_baselink_(gravity_in_baselink) {}

template <typename T>
bool GravityAlignmentCostFunctor::operator()(
    const T* const qwxyz_World_Baselink, T* residual) const {
  T g_in_B[3] = {T(gravity_in_baselink_[0]), T(gravity_in_baselink_[1]),
                 T(gravity_in_baselink_[2])};
  T q_W_B[4] = {T(qwxyz_World_Baselink[0]), T(qwxyz_World_Baselink[1]),
                T(qwxyz_World_Baselink[2]), T(qwxyz_World_Baselink[3])};
  T g_in_W_est[3];
  ceres::QuaternionRotatePoint(q_W_B, g_in_B, g_in_W_est);

  residual[0] = A_(0, 0) * g_in_W_est[0] + A_(0, 1) * g_in_W_est[1];
  residual[1] = A_(1, 0) * g_in_W_est[0] + A_(1, 1) * g_in_W_est[1];

  // // T q_W_I[4] = {T(qwxyz_World_Imu[0]), T(qwxyz_World_Imu[1]),
  // //               T(qwxyz_World_Imu[2]), T(qwxyz_World_Imu[3])};
  // // T q_I_W[4] = {T(qwxyz_Imu_World_[0]), T(qwxyz_Imu_World_[1]),
  // //               T(qwxyz_Imu_World_[2]), T(qwxyz_Imu_World_[3])};

  // // T q_diff[4];
  // // ceres::QuaternionProduct(q_I_W, q_W_I, q_diff);

  // // T g[3] = {T(g_nominal_[0]), T(g_nominal_[1]), T(g_nominal_[2])};

  // // T g_diff[3];
  // // ceres::QuaternionRotatePoint(q_diff, g, g_diff);

  // residual[0] = A_(0, 0) * g_diff[0] + A_(0, 1) * g_diff[1];
  // residual[1] = A_(1, 0) * g_diff[0] + A_(1, 1) * g_diff[1];
  return true;
}

} // namespace bs_constraints
