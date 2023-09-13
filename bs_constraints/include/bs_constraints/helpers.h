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
    OrientationAndPositionToTransformationMatrix(const T* const o_A_B,
                                                 const T* const p_A_B) {
  Eigen::Quaternion<T> q{o_A_B[0], o_A_B[1], o_A_B[2], o_A_B[3]};
  Eigen::Matrix<T, 4, 4> T_A_B = Eigen::Matrix<T, 4, 4>::Identity();
  T_A_B.block(0, 0, 3, 3) = q.toRotationMatrix();
  T_A_B(0, 3) = p_A_B[0];
  T_A_B(1, 3) = p_A_B[1];
  T_A_B(2, 3) = p_A_B[2];
  return T_A_B;
}

template <typename T>
Eigen::Matrix<T, 4, 4>
    InvertTransform(const Eigen::Matrix<T, 4, 4>& Transform) {
  Eigen::Matrix<T, 4, 4> T_inv = Eigen::Matrix<T, 4, 4>::Identity();
  T_inv.block(0, 0, 3, 3) = Transform.block(0, 0, 3, 3).transpose();
  T_inv.block(0, 3, 3, 1) =
      -Transform.block(0, 0, 3, 3).transpose() * Transform.block(0, 3, 3, 1);
  return T_inv;
}

} // namespace bs_constraints
