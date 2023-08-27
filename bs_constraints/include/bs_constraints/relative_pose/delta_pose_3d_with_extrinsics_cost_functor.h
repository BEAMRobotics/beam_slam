#pragma once

#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <ceres/rotation.h>

namespace bs_constraints {

/**
 * @brief Implements a cost function that models a difference between 3D pose
 * variables with extrinsics. This basically copies the
 * NormalDeltaPose3DCostFunctor with an extra transform for the extrinsics which
 * is not timestamped
 *
 */
class DeltaPose3DWithExtrinsicsCostFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Constructor
   *
   * @param[in] sqrt_info_ The square root information matrix used as the
   * residual weighting matrix (dx, dy, dz, dqx, dqy, dqz)
   * @param[in] d_Sensor1_Sensor2 The exposed pose difference between pose 1 and
   * pose 2 in order (dx, dy, dz, dqw, dqx, dqy, dqz) expressed in the sensor
   * frame
   */
  DeltaPose3DWithExtrinsicsCostFunctor(
      const fuse_core::Matrix6d& sqrt_info,
      const fuse_core::Vector7d& d_Sensor1_Sensor2);

  /**
   * @brief Compute the cost values/residuals using the provided
   * variable/parameter values
   */
  template <typename T>
  bool operator()(const T* const p_World_Baselink1_ptr,
                  const T* const o_World_Baselink1_ptr,
                  const T* const p_World_Baselink2_ptr,
                  const T* const o_World_Baselink2_ptr,
                  const T* const p_Baselink_Sensor_ptr,
                  const T* const o_Baselink_Sensor_ptr, T* residual) const;

private:
  /** The square root information matrix used as the residual weighting matrix
   */
  fuse_core::Matrix6d sqrt_info_;

  /** The measured difference between variable pose2 and variable pose1 in the
   * sensor frame. Note we invert the input difference here instead of inverting
   * the current estimate of the pose at each iteration of the optimizer */
  Eigen::Matrix4d T_Sensor2_Sensor1_Measured_{Eigen::Matrix4d::Identity()};

  template <typename T>
  Eigen::Matrix<T, 4, 4>
      InvertTransform(const Eigen::Matrix<T, 4, 4>& Transform) const;
};

DeltaPose3DWithExtrinsicsCostFunctor::DeltaPose3DWithExtrinsicsCostFunctor(
    const fuse_core::Matrix6d& sqrt_info,
    const fuse_core::Vector7d& d_Sensor1_Sensor2)
    : sqrt_info_(sqrt_info) {
  Eigen::Quaterniond q_S1_S2(d_Sensor1_Sensor2[3], d_Sensor1_Sensor2[4],
                             d_Sensor1_Sensor2[5], d_Sensor1_Sensor2[6]);
  Eigen::Matrix3d R_S1_S2 = q_S1_S2.toRotationMatrix();
  Eigen::Vector3d t_S1_S2(d_Sensor1_Sensor2[0], d_Sensor1_Sensor2[1],
                          d_Sensor1_Sensor2[2]);
  T_Sensor2_Sensor1_Measured_.block(0, 0, 3, 3) = R_S1_S2.transpose();
  T_Sensor2_Sensor1_Measured_.block(0, 3, 3, 1) =
      -R_S1_S2.transpose() * t_S1_S2;
}

template <typename T>
bool DeltaPose3DWithExtrinsicsCostFunctor::operator()(
    const T* const p_World_Baselink1_ptr, const T* const o_World_Baselink1_ptr,
    const T* const p_World_Baselink2_ptr, const T* const o_World_Baselink2_ptr,
    const T* const p_Baselink_Sensor_ptr, const T* const o_Baselink_Sensor_ptr,
    T* residual) const {
      
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_WB1(p_World_Baselink1_ptr);
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_WB2(p_World_Baselink2_ptr);
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_BS(p_Baselink_Sensor_ptr);
  
  Eigen::Map<const Eigen::Quaternion<T>> q_WB1(o_World_Baselink1_ptr);
  Eigen::Map<const Eigen::Quaternion<T>> q_WB2(o_World_Baselink2_ptr);
  Eigen::Map<const Eigen::Quaternion<T>> q_BS(o_Baselink_Sensor_ptr);
  
  Eigen::Matrix<T, 4, 4> T_BS = Eigen::Matrix<T, 4, 4>::Identity();
  T_BS.block(0, 0, 3, 3) = q_BS.toRotationMatrix();
  T_BS.block(0, 3, 3, 1) = t_BS;
  
  Eigen::Matrix<T, 4, 4> T_WB1 = Eigen::Matrix<T, 4, 4>::Identity();
  T_WB1.block(0, 0, 3, 3) = q_WB1.toRotationMatrix();
  T_WB1.block(0, 3, 3, 1) = t_WB1;

  Eigen::Matrix<T, 4, 4> T_WB2 = Eigen::Matrix<T, 4, 4>::Identity();
  T_WB2.block(0, 0, 3, 3) = q_WB2.toRotationMatrix();
  T_WB2.block(0, 3, 3, 1) = t_WB2;
  
  Eigen::Matrix<T, 4, 4> T_S1_S2_Estimated =
      InvertTransform<T>(T_BS) * InvertTransform<T>(T_WB1) * T_WB2 * T_BS;

  Eigen::Matrix<T, 4, 4> T_diff =
      T_S1_S2_Estimated * T_Sensor2_Sensor1_Measured_.cast<T>();
  
  // Compute the first three residual terms as (position_delta - b)
  residual[0] = T_diff(0,3);
  residual[1] = T_diff(1,3);
  residual[2] = T_diff(2,3);
  
  // compute the angle axis of the difference and set that to orientation
  // residuals
  Eigen::Matrix<T, 3, 3> R_diff = T_diff.block(0, 0, 3, 3);
  Eigen::AngleAxis<T> aa(R_diff);
  residual[3] = aa.axis()[0];
  residual[4] = aa.axis()[1];
  residual[5] = aa.axis()[2];
  
  // Map it to Eigen, and weight it
  Eigen::Map<Eigen::Matrix<T, 6, 1>> residual_map(residual);
  residual_map.applyOnTheLeft(sqrt_info_.template cast<T>());

  return true;
}

template <typename T>
Eigen::Matrix<T, 4, 4> DeltaPose3DWithExtrinsicsCostFunctor::InvertTransform(
    const Eigen::Matrix<T, 4, 4>& Transform) const {
  Eigen::Matrix<T, 4, 4> T_inv = Eigen::Matrix<T, 4, 4>::Identity();
  T_inv.block(0, 0, 3, 3) = Transform.block(0, 0, 3, 3).transpose();
  T_inv.block(0, 3, 3, 1) =
      -Transform.block(0, 0, 3, 3).transpose() * Transform.block(0, 3, 3, 1);
  return T_inv;
}

} // namespace bs_constraints
