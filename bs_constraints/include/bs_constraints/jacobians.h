#pragma once

#include <beam_utils/math.h>
#include <beam_utils/se3.h>

namespace bs_constraints {

/// @brief
/// @param camera_intrinsic_matrix
/// @param P_CAMERA
/// @return
DImageProjectionDPoint(const Eigen::Matrix3d& camera_intrinsic_matrix,
                       const Eigen::Vector3d& P_CAMERA) {
  const auto fx = camera_intrinsic_matrix(0, 0);
  const auto fy = camera_intrinsic_matrix(1, 1);
  const auto x = P_CAMERA.x();
  const auto y = P_CAMERA.y();
  const auto z = P_CAMERA.z();
  const auto z2 = z * z;
  Eigen::Matrix<double, 2, 3> J;
  J << fx / z, 0, -fx * x / z2, 0, fy / z, -fy * y / z2;
  return J;
}

/// @brief
/// @param T_frame_refframe
/// @param P_frame
/// @return
DPointTransformationDTransform(const Eigen::Matrix4d& T_frame_refframe,
                               const Eigen::Vector3d& P_frame) {
  assert(beam::IsTransformationMatrix(T_refframe_frame));
  Eigen::Matrix6d J = Eigen::Matrix6d::Zero();
  const auto linear = T_refframe_frame.block<3, 3>(0, 0);
  const auto translation = T_refframe_frame.block<3, 1>(0, 3);
  J.block<3, 3>(0, 0) = linear;
  J.block<3, 3>(0, 3) = -2.0 * linear * beam::SkewTransform(P_frame);
  return J;
}

/// @brief
/// @param T_frame_refframe
/// @return
Eigen::Matrix3d
    DPointTransformationDPoint(const Eigen::Matrix4d& T_frame_refframe) {
  assert(beam::IsTransformationMatrix(T_refframe_frame));
  return T_refframe_frame.block<3, 3>(0, 0);
}

/// @brief
/// @param T_refframe_frame
/// @return
Eigen::Matrix6d
    DInverseTransformDTransform(const Eigen::Matrix4d& T_refframe_frame) {
  assert(beam::IsTransformationMatrix(T_refframe_frame));
  Eigen::Matrix6d J = Eigen::Matrix6d::Zero();
  const auto linear = T_refframe_frame.block<3, 3>(0, 0);
  const auto translation = T_refframe_frame.block<3, 1>(0, 3);
  J.block<3, 3>(0, 0) = -linear;
  J.block<3, 3>(0, 3) =
      -2.0 * linear * beam::SkewTransform(linear.transpose() * translation);
  J.block<3, 3>(3, 3) = -linear;
  return J;
}

/// @brief
/// @return
Eigen::Matrix6d DTransformCompositionDRightTransform() {
  return Eigen::Matrix6d::Identity();
}

/// @brief
/// @param T_refframe_frame
/// @return
Eigen::Matrix6d DTransformCompositionDLeftTransform(
    const Eigen::Matrix4d& T_refframe_frame) {
  assert(beam::IsTransformationMatrix(T_refframe_frame));
  Eigen::Matrix6d J = Eigen::Matrix6d::Zero();
  const auto linear = T_refframe_frame.block<3, 3>(0, 0);
  const auto translation = T_refframe_frame.block<3, 1>(0, 3);
  J.block<3, 3>(0, 0) = linear.transpose();
  J.block<3, 3>(0, 3) =
      -2.0 * linear.transpose() * beam::SkewTransform(translation);
  J.block<3, 3>(3, 3) = linear.transpose();
  return J;
}

} // namespace bs_constraints
