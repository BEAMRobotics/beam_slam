#pragma once

#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <ceres/rotation.h>

namespace bs_constraints {

template <typename T>
Eigen::Matrix<T, 4, 4>
    OrientationAndPositionToTransformationMatrix(const T* const p_A_B,
                                                 const T* const o_A_B) {
  T R[9];
  ceres::QuaternionToRotation(o_A_B, R);
  Eigen::Matrix<T, 3, 3> R_A_B = Eigen::Map<Eigen::Matrix<T, 3, 3> >(R, 3, 3);
  Eigen::Matrix<T, 3, 1> t_A_B(p_A_B[0], p_A_B[1], p_A_B[2]);
  Eigen::Matrix<T, 4, 4> T_A_B = Eigen::Matrix<T, 4, 4>::Identity();
  T_A_B.block(0, 0, 3, 3) = R_A_B;
  T_A_B.block(0, 3, 3, 1) = t_A_B;
  return T_A_B;
}

template <typename T>
Eigen::Matrix<T, 4, 4> InvertTransform(const Eigen::Matrix<T, 4, 4>& T_A_B) {
  Eigen::Matrix<T, 3, 3> R_A_B = T_A_B.block(0, 0, 3, 3);
  Eigen::Matrix<T, 3, 1> t_A_B = T_A_B.block(0, 3, 3, 1);
  Eigen::Matrix<T, 4, 4> T_B_A = Eigen::Matrix<T, 4, 4>::Identity();
  T_B_A.block(0, 0, 3, 3) = R_A_B.transpose();
  T_B_A.block(0, 3, 3, 1) = -R_A_B.transpose() * t_A_B;
  return T_B_A;
}

} // namespace bs_constraints
