#pragma once

#include <beam_utils/math.h>
#include <beam_utils/se3.h>

namespace bs_constraints {

/// @brief
/// @param camera_intrinsic_matrix
/// @param P_CAMERA
/// @return
Eigen::Matrix<double, 2, 3>
    DImageProjectionDPoint(const Eigen::Matrix3d& camera_intrinsic_matrix,
                           const Eigen::Vector3d& P_CAMERA);

/// @brief
/// @param T_frame_refframe
/// @param P_frame
/// @return
Eigen::Matrix<double, 3, 6>
    DPointTransformationDTransform(const Eigen::Matrix4d& T_frame_refframe,
                                   const Eigen::Vector3d& P_frame);

/// @brief
/// @param T_refframe_frame
/// @return
Eigen::Matrix3d
    DPointTransformationDPoint(const Eigen::Matrix4d& T_refframe_frame);

/// @brief
/// @param T_refframe_frame
/// @return
Eigen::Matrix<double, 6, 6>
    DInverseTransformDTransform(const Eigen::Matrix4d& T_refframe_frame);

/// @brief
/// @return
Eigen::Matrix<double, 6, 6> DTransformCompositionDRightTransform();

/// @brief
/// @param T_refframe_frame
/// @return
Eigen::Matrix<double, 6, 6> DTransformCompositionDLeftTransform(
    const Eigen::Matrix4d& T_refframe_frame);

} // namespace bs_constraints
