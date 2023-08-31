#pragma once

#include <fuse_constraints/normal_delta_pose_3d_cost_functor.h>
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
  bool operator()(const T* const p_World_Baselink1,
                  const T* const o_World_Baselink1,
                  const T* const p_World_Baselink2,
                  const T* const o_World_Baselink2,
                  const T* const p_Baselink_Sensor,
                  const T* const o_Baselink_Sensor, T* residual) const;

private:
  fuse_constraints::NormalDeltaPose3DCostFunctor
      normal_delta_pose_cost_functor_;

  template <typename T>
  void TransformPoseToSensorFrame(const T* const p_World_Baselink,
                                  const T* const o_World_Baselink,
                                  const T* const p_Baselink_Sensor,
                                  const T* const o_Baselink_Sensor,
                                  T* p_World_Sensor, T* o_World_Sensor) const;
};

DeltaPose3DWithExtrinsicsCostFunctor::DeltaPose3DWithExtrinsicsCostFunctor(
    const fuse_core::Matrix6d& sqrt_info,
    const fuse_core::Vector7d& d_Sensor1_Sensor2)
    : normal_delta_pose_cost_functor_(sqrt_info, d_Sensor1_Sensor2) {}

template <typename T>
bool DeltaPose3DWithExtrinsicsCostFunctor::operator()(
    const T* const p_World_Baselink1, const T* const o_World_Baselink1,
    const T* const p_World_Baselink2, const T* const o_World_Baselink2,
    const T* const p_Baselink_Sensor, const T* const o_Baselink_Sensor,
    T* residual) const {
  T p_World_Sensor1[3];
  T o_World_Sensor1[4];
  TransformPoseToSensorFrame<T>(p_World_Baselink1, o_World_Baselink1,
                                p_Baselink_Sensor, o_Baselink_Sensor,
                                p_World_Sensor1, o_World_Sensor1);

  T p_World_Sensor2[3];
  T o_World_Sensor2[4];
  TransformPoseToSensorFrame<T>(p_World_Baselink2, o_World_Baselink2,
                                p_Baselink_Sensor, o_Baselink_Sensor,
                                p_World_Sensor2, o_World_Sensor2);

  return normal_delta_pose_cost_functor_(p_World_Sensor1, o_World_Sensor1,
                                         p_World_Sensor2, o_World_Sensor2,
                                         &residual[0]);
}

/**
 * We want to calculate the pose of each sensor frame (1 & 2) to call the
 * regular relative pose functor. We can calculate this for each pose as
 * follows:
 *
 * R_W_S = R_W_B * R_B_S
 * t_W_S = R_W_B * t_B_S + t_W_B
 *
 */
template <typename T>
void DeltaPose3DWithExtrinsicsCostFunctor::TransformPoseToSensorFrame(
    const T* const p_World_Baselink, const T* const o_World_Baselink,
    const T* const p_Baselink_Sensor, const T* const o_Baselink_Sensor,
    T* p_World_Sensor, T* o_World_Sensor) const {
  ceres::QuaternionProduct(o_World_Baselink, o_Baselink_Sensor, o_World_Sensor);
  T World_t_Baselink_Sensor[3]; // sensor to baselink, expressed in world
  ceres::QuaternionRotatePoint(o_World_Baselink, p_Baselink_Sensor,
                               World_t_Baselink_Sensor);
  p_World_Sensor[0] = World_t_Baselink_Sensor[0] + p_World_Baselink[0];
  p_World_Sensor[1] = World_t_Baselink_Sensor[1] + p_World_Baselink[1];
  p_World_Sensor[2] = World_t_Baselink_Sensor[2] + p_World_Baselink[2];
}
} // namespace bs_constraints
