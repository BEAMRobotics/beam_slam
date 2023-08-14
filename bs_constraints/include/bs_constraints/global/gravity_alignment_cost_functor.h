#pragma once

#include <Eigen/Core>
#include <fuse_constraints/normal_prior_orientation_3d_cost_functor.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

namespace bs_constraints { namespace global {

/**
 * @brief Create an absolute constraint on orienation to align with gravity.
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
 * |e_x| = [ (R_W_I')^(-1) * R'_W_I * g' ]
 * |e_y|
 *
 * g' is the nominal gravity vector [0,0,-1], R'_W_I is the measured
 * rotation from IMU to World, and R_W_I is the current estimate of rotation
 * from IMU to World. Note we ignore the z term since we only care about the
 * deviation in x and y directions (roll & pitch)
 *
 */
class GravityAlignmentCostFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  GravityAlignmentCostFunctor(
      const Eigen::Matrix<double, 2, 2>& A,
      const Eigen::Matrix<double, 4, 1>& qwxyz_Imu_World)
      : A_(A), qwxyz_Imu_World_(qwxyz_Imu_World) {
    qwxyz_Imu_World_[0] = qwxyz_Imu_World[0];
    qwxyz_Imu_World_[1] = qwxyz_Imu_World[1];
    qwxyz_Imu_World_[2] = qwxyz_Imu_World[2];
    qwxyz_Imu_World_[3] = qwxyz_Imu_World[3];
    g_nominal_[0] = 0;
    g_nominal_[1] = 0;
    g_nominal_[2] = -1;
  }

  template <typename T>
  bool operator()(const T* const qwxyz_World_Imu, T* residual) const {
    T qwxyz_diff[4];
    ceres::QuaternionProduct(qwxyz_Imu_World_, qwxyz_World_Imu, qwxyz_diff);
    T g_diff[3];
    ceres::QuaternionRotatePoint(qwxyz_diff, g_nominal_, g_diff);
    residual[0] = A_(0, 0) * g_diff[0] + A_(0, 1) * g_diff[1];
    residual[1] = A_(1, 0) * g_diff[0] + A_(1, 1) * g_diff[1];
  }

private:
  Eigen::Matrix<double, 2, 2> A_;
  double qwxyz_Imu_World_[4];
  double g_nominal_[4];

  fuse_constraints::NormalPriorOrientation3DCostFunctor orientation_functor_;
};

}} // namespace bs_constraints::global
