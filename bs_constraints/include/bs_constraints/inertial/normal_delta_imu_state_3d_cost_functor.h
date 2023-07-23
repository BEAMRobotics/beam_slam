#pragma once

#include <ceres/rotation.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/util.h>

#include <bs_common/imu_state.h>
#include <bs_common/preintegrator.h>
#include <bs_common/utils.h>

namespace bs_constraints { namespace inertial {

/**
 * @brief Implements a cost function that models a difference between 3D
 * IMU states according to DOI: 10.15607/RSS.2015.XI.006
 */
class NormalDeltaImuState3DCostFunctor {
public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Constructor
   * @param imu_state_i current IMU state
   * @param pre_integrator preintegrator class containing IMU data between new
   * and current IMU states
   */
  NormalDeltaImuState3DCostFunctor(
      const bs_common::ImuState& imu_state_i,
      const std::shared_ptr<bs_common::PreIntegrator> pre_integrator,
      const double info_weight);

  /**
   * @brief Compute the cost values/residuals using the provided
   * variable/parameter values
   */
  template <typename T>
  bool operator()(const T* const orientation1, const T* const position1,
                  const T* const velocity1, const T* const gyrobias1,
                  const T* const accelbias1, const T* const orientation2,
                  const T* const position2, const T* const velocity2,
                  const T* const gyrobias2, const T* const accelbias2,
                  T* residual) const;

private:
  bs_common::ImuState imu_state_i_;
  std::shared_ptr<bs_common::PreIntegrator> pre_integrator_;
  Eigen::Matrix<double, 15, 15> A_; //!< The residual weighting matrix
};

NormalDeltaImuState3DCostFunctor::NormalDeltaImuState3DCostFunctor(
    const bs_common::ImuState& imu_state_i,
    const std::shared_ptr<bs_common::PreIntegrator> pre_integrator,
    const double info_weight)
    : imu_state_i_(imu_state_i),
      pre_integrator_(pre_integrator),
      A_(info_weight * pre_integrator->delta.sqrt_inv_cov) {}

template <typename T>
bool NormalDeltaImuState3DCostFunctor::operator()(
    const T* const orientation1, const T* const position1,
    const T* const velocity1, const T* const gyrobias1,
    const T* const accelbias1, const T* const orientation2,
    const T* const position2, const T* const velocity2,
    const T* const gyrobias2, const T* const accelbias2, T* residual) const {
  // map input to templated
  Eigen::Quaternion<T> q_i(orientation1[0], orientation1[1], orientation1[2],
                           orientation1[3]);
  Eigen::Matrix<T, 3, 1> p_i(position1[0], position1[1], position1[2]);
  Eigen::Matrix<T, 3, 1> v_i(velocity1[0], velocity1[1], velocity1[2]);
  Eigen::Matrix<T, 3, 1> bg_i(gyrobias1[0], gyrobias1[1], gyrobias1[2]);
  Eigen::Matrix<T, 3, 1> ba_i(accelbias1[0], accelbias1[1], accelbias1[2]);

  Eigen::Quaternion<T> q_j(orientation2[0], orientation2[1], orientation2[2],
                           orientation2[3]);
  Eigen::Matrix<T, 3, 1> p_j(position2[0], position2[1], position2[2]);
  Eigen::Matrix<T, 3, 1> v_j(velocity2[0], velocity2[1], velocity2[2]);
  Eigen::Matrix<T, 3, 1> bg_j(gyrobias2[0], gyrobias2[1], gyrobias2[2]);
  Eigen::Matrix<T, 3, 1> ba_j(accelbias2[0], accelbias2[1], accelbias2[2]);

  // map preintegrator to templated
  T dt = static_cast<T>(pre_integrator_->delta.t.toSec());
  Eigen::Quaternion<T> dq = pre_integrator_->delta.q.cast<T>();
  Eigen::Matrix<T, 3, 1> dp = pre_integrator_->delta.p.cast<T>();
  Eigen::Matrix<T, 3, 1> dv = pre_integrator_->delta.v.cast<T>();
  Eigen::Matrix<T, 3, 1> dbg = bg_i - imu_state_i_.GyroBiasVec().cast<T>();
  Eigen::Matrix<T, 3, 1> dba = ba_i - imu_state_i_.AccelBiasVec().cast<T>();
  Eigen::Matrix<T, 3, 3> dq_dbg = pre_integrator_->jacobian.dq_dbg.cast<T>();
  Eigen::Matrix<T, 3, 3> dp_dbg = pre_integrator_->jacobian.dp_dbg.cast<T>();
  Eigen::Matrix<T, 3, 3> dp_dba = pre_integrator_->jacobian.dp_dba.cast<T>();
  Eigen::Matrix<T, 3, 3> dv_dbg = pre_integrator_->jacobian.dv_dbg.cast<T>();
  Eigen::Matrix<T, 3, 3> dv_dba = pre_integrator_->jacobian.dv_dba.cast<T>();

  // map gravity to templated
  Eigen::Matrix<T, 3, 1> G = GRAVITY_WORLD.cast<T>();
  
  Eigen::Matrix<T, 3, 1> q_tmp = dq_dbg * dbg;
  Eigen::Quaternion<T> q_corrected = dq * bs_common::DeltaQ(q_tmp);
  Eigen::Matrix<T, 3, 1> p_corrected = dp + dp_dbg * dbg + dp_dba * dba;
  Eigen::Matrix<T, 3, 1> v_corrected = dv + dv_dbg * dbg + dv_dba * dba;

  // calculate orientation residual
  Eigen::Matrix<T, 3, 1> res_q =
      static_cast<T>(2) * (q_corrected.inverse() * (q_i.inverse() * q_j)).vec();

  // calculate position residual
  Eigen::Matrix<T, 3, 1> res_p =
      q_i.conjugate() *
          (p_j - p_i - dt * v_i - static_cast<T>(0.5) * dt * dt * G) -
      p_corrected;

  // calculate velocity residual
  Eigen::Matrix<T, 3, 1> res_v =
      q_i.conjugate() * (v_j - v_i - dt * G) - v_corrected;

  // calculate bias residuals
  Eigen::Matrix<T, 3, 1> res_bg = bg_j - bg_i;
  Eigen::Matrix<T, 3, 1> res_ba = ba_j - ba_i;

  residual[0] = res_q[0];
  residual[1] = res_q[1];
  residual[2] = res_q[2];
  residual[3] = res_p[0];
  residual[4] = res_p[1];
  residual[5] = res_p[2];
  residual[6] = res_v[0];
  residual[7] = res_v[1];
  residual[8] = res_v[2];
  residual[9] = res_bg[0];
  residual[10] = res_bg[1];
  residual[11] = res_bg[2];
  residual[12] = res_ba[0];
  residual[13] = res_ba[1];
  residual[14] = res_ba[2];

  // Map residuals to Eigen, and weight it
  Eigen::Map<Eigen::Matrix<T, 15, 1>> residual_map(residual);
  residual_map.applyOnTheLeft(A_.template cast<T>());

  return true;
}

}} // namespace bs_constraints::inertial
