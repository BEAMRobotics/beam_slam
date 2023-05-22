#pragma once

#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <ceres/rotation.h>

namespace bs_constraints {

/// @brief
/// @param q
/// @param pert
/// @return
Eigen::Quaterniond SO3BoxPlus(const Eigen::Quaterniond& q,
                              const Eigen::Vector3d& pert);

/// @brief
/// @param q1
/// @param q2
/// @return
Eigen::Vector3d SO3BoxMinus(const Eigen::Quaterniond& q1,
                            const Eigen::Quaterniond& q2);

/// @brief
/// @param T
/// @param pert
/// @return
Eigen::Matrix<double, 7, 1>
    TranslationSO3BoxPlus(const Eigen::Matrix<double, 7, 1>& T,
                          const Eigen::Matrix<double, 6, 1>& pert);

/// @brief Box Minus for a transform that is parameterized as a seperate
/// Translation and Orientation
/// @param T1
/// @param T2
/// @return
Eigen::Matrix<double, 6, 1>
    TranslationSO3BoxMinus(const Eigen::Matrix<double, 7, 1>& T1,
                           const Eigen::Matrix<double, 7, 1>& T2);

/// @brief
/// @param T
/// @param pert
/// @return
Eigen::Matrix<double, 7, 1> SE3BoxPlus(const Eigen::Matrix<double, 7, 1>& T,
                                       const Eigen::Matrix<double, 6, 1>& pert);

/// @brief Box Minus for a transform that is parameterized as a single SE3
/// transform
/// @param T1
/// @param T2
/// @return
Eigen::Matrix<double, 6, 1> SE3BoxMinus(const Eigen::Matrix<double, 7, 1>& T1,
                                        const Eigen::Matrix<double, 7, 1>& T2);

/// @brief
/// @param q
/// @return
Eigen::Matrix<double, 4, 3> PlusJacobian(const Eigen::Quaterniond& q);

/// @brief
/// @param q
/// @return
Eigen::Matrix<double, 3, 4> MinusJacobian(const Eigen::Quaterniond& q);

/// @brief
/// @param R
/// @param point
/// @return
Eigen::Matrix3d DPointRotationDRotation(const Eigen::Matrix3d& R,
                                        const Eigen::Vector3d& point);

/// @brief
/// @param R
/// @param point
/// @return
Eigen::Matrix3d DPointRotationDPoint(const Eigen::Matrix3d& R,
                                     const Eigen::Vector3d& point);

/// @brief
/// @param R
/// @return
Eigen::Matrix3d DInverseRotationDRotation(const Eigen::Matrix3d& R);

/// @brief
/// @param R_left
/// @param R_right
/// @return
Eigen::Matrix3d
    DRotationCompositionDLeftRotation(const Eigen::Matrix3d& R_left,
                                      const Eigen::Matrix3d& R_right);

/// @brief
/// @param R_left
/// @param R_right
/// @return
Eigen::Matrix3d
    DRotationCompositionDRightRotation(const Eigen::Matrix3d& R_left,
                                       const Eigen::Matrix3d& R_right);

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

/// @brief Computes jacobian of T_left * T_right wrt T_right
/// @param T_left left transform
/// @param T_right right transform
/// @return
Eigen::Matrix<double, 6, 6>
    DTransformCompositionDRightTransform(const Eigen::Matrix4d& T_left,
                                         const Eigen::Matrix4d& T_right);

/// @brief Computes jacobian of T_left * T_right wrt T_left
/// @param T_left left transform
/// @param T_right right transform
/// @return Jacobian with position first, then quaternion
Eigen::Matrix<double, 6, 6>
    DTransformCompositionDLeftTransform(const Eigen::Matrix4d& T_left,
                                        const Eigen::Matrix4d& T_right);

} // namespace bs_constraints
