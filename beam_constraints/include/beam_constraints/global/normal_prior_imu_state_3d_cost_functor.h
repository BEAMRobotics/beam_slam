#pragma once

#include <fuse_constraints/normal_prior_orientation_3d_cost_functor.h>
#include <fuse_core/eigen.h>
#include <fuse_core/macros.h>
#include <fuse_core/util.h>
#include <Eigen/Core>

namespace beam_constraints { namespace global {

/**
 * @brief Create a prior cost function on the 3D imu state at once.
 *
 * The Ceres::NormalPrior cost function only supports a single variable. This is
 * a convenience cost function that applies a prior constraint on both 3D imu
 * state variables at once.
 *
 * The cost function is of the form:
 *
 *   cost(x) = || A * [  AngleAxis(b(0:3)^-1 * q) ] ||^2
 *             ||     [  v - b(4:6)               ] ||
 *						 ||     [  p - b(7:9)               ] ||
 *             ||     [  ba - b(10:12)            ] ||
 *             ||     [  bg - b(13:15)            ] ||
 *
 * where, the matrix A and the vector b are fixed, q is the orientation
 * variable, v is the linear velocity variable, p is the position variable, ba
 * is the acceleration bias variable, and bg is the gyroscope bias variable.
 * Note that the covariance submatrix for the quaternion is 3x3, representing
 * errors in the orientation local parameterization tangent space.
 */
class NormalPriorImuState3DCostFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  NormalPriorImuState3DCostFunctor(const Eigen::Matrix<double, 15, 15>& A,
                                   const Eigen::Matrix<double, 16, 1>& b);

  template <typename T>
  bool operator()(const T* const orientation, const T* const velocity,
                  const T* const position, const T* const accelbias,
                  const T* const gyrobias, T* residual) const;

private:
  Eigen::Matrix<double, 15, 15> A_;
  Eigen::Matrix<double, 16, 1> b_;

  fuse_constraints::NormalPriorOrientation3DCostFunctor orientation_functor_;
};

NormalPriorImuState3DCostFunctor::NormalPriorImuState3DCostFunctor(
    const Eigen::Matrix<double, 15, 15>& A,
    const Eigen::Matrix<double, 16, 1>& b)
    : A_(A),
      b_(b),
      orientation_functor_(fuse_core::Matrix3d::Identity(), b_.head<4>()) {}

template <typename T>
bool NormalPriorImuState3DCostFunctor::operator()(const T* const orientation,
                                                  const T* const velocity,
                                                  const T* const position,
                                                  const T* const accelbias,
                                                  const T* const gyrobias,
                                                  T* residual) const {
  // Use the 3D orientation cost functor to compute the orientation delta
  orientation_functor_(orientation, &residual[0]);

  // Compute the velocity error
  residual[4] = velocity[0] - T(b_(4));
  residual[5] = velocity[1] - T(b_(5));
  residual[6] = velocity[2] - T(b_(6));

  // Compute the position error
  residual[7] = position[0] - T(b_(7));
  residual[8] = position[1] - T(b_(8));
  residual[9] = position[2] - T(b_(9));

  // Compute the acceleration bias error
  residual[10] = accelbias[0] - T(b_(10));
  residual[11] = accelbias[1] - T(b_(11));
  residual[12] = accelbias[2] - T(b_(12));

  // Compute the gyroscope bias error
  residual[13] = gyrobias[0] - T(b_(13));
  residual[14] = gyrobias[1] - T(b_(14));
  residual[14] = gyrobias[2] - T(b_(15));

  // Scale the residuals by the square root information matrix to account for
  // the measurement uncertainty.
  Eigen::Map<Eigen::Matrix<T, 15, 1>> residual_map(residual);
  residual_map.applyOnTheLeft(A_.template cast<T>());

  return true;
}

}}  // namespace beam_constraints::global
