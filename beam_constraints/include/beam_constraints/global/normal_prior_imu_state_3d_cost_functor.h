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
 *   cost(x) = || A * [  p - b(0:2)               ] ||^2
 *						 ||     [  v - b(3:5)
 *] ||
 *             ||     [  AngleAxis(b(6:9)^-1 * q) ] ||
 *
 * where, the matrix A and the vector b are fixed, p is the position variable, v
 * is the velocity variable and q is the orientation variable. Note that the
 * covariance submatrix for the quaternion is 3x3, representing errors in the
 * orientation local parameterization tangent space.
 */
class NormalPriorImuState3DCostFunctor {
 public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  NormalPriorImuState3DCostFunctor(const fuse_core::Matrix9d& A,
                                   const Eigen::Matrix<double, 10, 1>& b);

  template <typename T>
  bool operator()(const T* const position, const T* const velocity,
                  const T* const orientation, T* residual) const;

 private:
  fuse_core::Matrix9d A_;
  Eigen::Matrix<double, 10, 1> b_;

  fuse_constraints::NormalPriorOrientation3DCostFunctor orientation_functor_;
};

NormalPriorImuState3DCostFunctor::NormalPriorImuState3DCostFunctor(
    const fuse_core::Matrix9d& A, const Eigen::Matrix<double, 10, 1>& b)
    : A_(A),
      b_(b),
      orientation_functor_(fuse_core::Matrix3d::Identity(), b_.tail<4>()) {}

template <typename T>
bool NormalPriorImuState3DCostFunctor::operator()(const T* const position,
                                                  const T* const velocity,
                                                  const T* const orientation,
                                                  T* residual) const {
  // Compute the position error
  residual[0] = position[0] - T(b_(0));
  residual[1] = position[1] - T(b_(1));
  residual[2] = position[2] - T(b_(2));

  // Compute the velocity error
  residual[3] = velocity[0] - T(b_(3));
  residual[4] = velocity[1] - T(b_(4));
  residual[5] = velocity[2] - T(b_(5));

  // Use the 3D orientation cost functor to compute the orientation delta
  orientation_functor_(orientation, &residual[6]);

  // Scale the residuals by the square root information matrix to account for
  // the measurement uncertainty.
  Eigen::Map<Eigen::Matrix<T, 9, 1>> residual_map(residual);
  residual_map.applyOnTheLeft(A_.template cast<T>());

  return true;
}

}}  // namespace beam_constraints::global
