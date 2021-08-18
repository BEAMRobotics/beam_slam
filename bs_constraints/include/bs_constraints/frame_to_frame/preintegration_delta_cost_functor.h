#pragma once

#include <ceres/ceres.h>

#include <bs_common/preintegrator.h>
#include <bs_models/frame_to_frame/imu_state.h>

#include <beam_utils/math.h>

class PreIntegrationDeltaCost
    : public ceres::SizedCostFunction<15, 4, 3, 3, 3, 3, 4, 3, 3, 3, 3> {
 public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Constructor
   * @param imu_state_i current IMU state
   * @param imu_state_j new IMU state
   * @param pre_integrator preintegrator class containing IMU data between new
   * and current IMU states
   */
  PreIntegrationDeltaCost(
      const ImuState &imu_state_i, const ImuState &imu_state_j,
      const std::shared_ptr<bs_common::PreIntegrator> pre_integrator)
      : imu_state_i_(imu_state_i),
        imu_state_j_(imu_state_j),
        pre_integrator_(pre_integrator) {}

  /**
   * @brief Override for Cere's Evaluate function. See Cere's documentation for
   * parameter description
   */
  bool Evaluate(const double *const *parameters, double *residuals,
                double **jacobians) const override {
    // get current states
    Eigen::Map<const Eigen::Quaterniond> q_i(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> p_i(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> v_i(parameters[2]);
    Eigen::Map<const Eigen::Vector3d> bg_i(parameters[3]);
    Eigen::Map<const Eigen::Vector3d> ba_i(parameters[4]);

    // get new states
    Eigen::Map<const Eigen::Quaterniond> q_j(parameters[5]);
    Eigen::Map<const Eigen::Vector3d> p_j(parameters[6]);
    Eigen::Map<const Eigen::Vector3d> v_j(parameters[7]);
    Eigen::Map<const Eigen::Vector3d> bg_j(parameters[8]);
    Eigen::Map<const Eigen::Vector3d> ba_j(parameters[9]);

    // get state deltas
    const double &dt =
        ros::Duration(imu_state_j.Stamp() - imu_state_i.Stamp()).toSec();
    const Eigen::Quaterniond &dq = pre_integrator_->delta.q;
    const Eigen::Vector3d &dp = pre_integrator_->delta.p;
    const Eigen::Vector3d &dv = pre_integrator_->delta.v;
    const Eigen::Vector3d dbg = bg_i - imu_state_i.AccelGyroVec();
    const Eigen::Vector3d dba = ba_i - imu_state_i.AccelBiasVec();

    // get jacobians from preintegrator
    const Eigen::Matrix3d &dq_dbg = pre_integrator_->jacobian.dq_dbg;
    const Eigen::Matrix3d &dp_dbg = pre_integrator_->jacobian.dp_dbg;
    const Eigen::Matrix3d &dp_dba = pre_integrator_->jacobian.dp_dba;
    const Eigen::Matrix3d &dv_dbg = pre_integrator_->jacobian.dv_dbg;
    const Eigen::Matrix3d &dv_dba = pre_integrator_->jacobian.dv_dba;

    // calculate residuals
    Eigen::Map<Eigen::Matrix<double, 15, 1>> r(residuals);
    r.segment<3>(ES_Q) = beam::QToLieAlgebra(
        (dq * beam::LieAlgebraToQ(dq_dbg * dbg)).conjugate() * q_i.conjugate() *
        q_j);
    r.segment<3>(ES_P) = q_i.conjugate() * (p_j - p_i - dt * v_i -
                                            0.5 * dt * dt * GRAVITY_WORLD) -
                         (dp + dp_dbg * dbg + dp_dba * dba);
    r.segment<3>(ES_V) = q_i.conjugate() * (v_j - v_i - dt * GRAVITY_WORLD) -
                         (dv + dv_dbg * dbg + dv_dba * dba);
    r.segment<3>(ES_BG) = bg_j - bg_i;
    r.segment<3>(ES_BA) = ba_j - ba_i;

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> dr_dq_i(
            jacobians[0]);
        dr_dq_i.setZero();
        dr_dq_i.block<3, 3>(ES_Q, 0) =
            -beam::RightJacobianOfSO3(r.segment<3>(ES_Q)).inverse() *
            q_j.conjugate().matrix() * q_i.matrix();
        dr_dq_i.block<3, 3>(ES_P, 0) = beam::SkewTransform(
            q_i.conjugate() *
            (p_j - p_i - dt * v_i - 0.5 * dt * dt * GRAVITY_WORLD));
        dr_dq_i.block<3, 3>(ES_V, 0) = beam::SkewTransform(
            q_i.conjugate() * (v_j - v_i - dt * GRAVITY_WORLD));
        dr_dq_i = pre_integrator_->delta.sqrt_inv_cov * dr_dq_i;
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> dr_dp_i(
            jacobians[1]);
        dr_dp_i.setZero();
        dr_dp_i.block<3, 3>(ES_P, 0) = -q_i.conjugate().matrix();
        dr_dp_i = pre_integrator_->delta.sqrt_inv_cov * dr_dp_i;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> dr_dv_i(
            jacobians[2]);
        dr_dv_i.setZero();
        dr_dv_i.block<3, 3>(ES_P, 0) = -dt * q_i.conjugate().matrix();
        dr_dv_i.block<3, 3>(ES_V, 0) = -q_i.conjugate().matrix();
        dr_dv_i = pre_integrator_->delta.sqrt_inv_cov * dr_dv_i;
      }
      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> dr_dbg_i(
            jacobians[3]);
        dr_dbg_i.setZero();
        dr_dbg_i.block<3, 3>(ES_Q, 0) =
            -beam::RightJacobianOfSO3(r.segment<3>(ES_Q)).inverse() *
            beam::LieAlgebraToQ(r.segment<3>(ES_Q)).conjugate().matrix() *
            beam::RightJacobianOfSO3(dq_dbg * dbg) * dq_dbg;
        dr_dbg_i.block<3, 3>(ES_P, 0) = -dp_dbg;
        dr_dbg_i.block<3, 3>(ES_V, 0) = -dv_dbg;
        dr_dbg_i.block<3, 3>(ES_BG, 0) = -Eigen::Matrix3d::Identity();
        dr_dbg_i = pre_integrator_->delta.sqrt_inv_cov * dr_dbg_i;
      }
      if (jacobians[4]) {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> dr_dba_i(
            jacobians[4]);
        dr_dba_i.setZero();
        dr_dba_i.block<3, 3>(ES_P, 0) = -dp_dba;
        dr_dba_i.block<3, 3>(ES_V, 0) = -dv_dba;
        dr_dba_i.block<3, 3>(ES_BA, 0) = -Eigen::Matrix3d::Identity();
        dr_dba_i = pre_integrator_->delta.sqrt_inv_cov * dr_dba_i;
      }
      if (jacobians[5]) {
        Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> dr_dq_j(
            jacobians[5]);
        dr_dq_j.setZero();
        dr_dq_j.block<3, 3>(ES_Q, 0) =
            beam::RightJacobianOfSO3(r.segment<3>(ES_Q)).inverse();
        dr_dq_j.block<3, 3>(ES_P, 0) = -q_i.conjugate().matrix() *
                                       q_j.matrix() *
                                       beam::SkewTransform(imu_j.p_cs);
        dr_dq_j = pre_integrator_->delta.sqrt_inv_cov * dr_dq_j;
      }
      if (jacobians[6]) {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> dr_dp_j(
            jacobians[6]);
        dr_dp_j.setZero();
        dr_dp_j.block<3, 3>(ES_P, 0) = q_i.conjugate().matrix();
        dr_dp_j = pre_integrator_->delta.sqrt_inv_cov * dr_dp_j;
      }
      if (jacobians[7]) {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> dr_dv_j(
            jacobians[7]);
        dr_dv_j.setZero();
        dr_dv_j.block<3, 3>(ES_V, 0) = q_i.conjugate().matrix();
        dr_dv_j = pre_integrator_->delta.sqrt_inv_cov * dr_dv_j;
      }
      if (jacobians[8]) {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> dr_dbg_j(
            jacobians[8]);
        dr_dbg_j.setZero();
        dr_dbg_j.block<3, 3>(ES_BG, 0).setIdentity();
        dr_dbg_j = pre_integrator_->delta.sqrt_inv_cov * dr_dbg_j;
      }
      if (jacobians[9]) {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> dr_dba_j(
            jacobians[9]);
        dr_dba_j.setZero();
        dr_dba_j.block<3, 3>(ES_BA, 0).setIdentity();
        dr_dba_j = pre_integrator_->delta.sqrt_inv_cov * dr_dba_j;
      }
    }

    r = pre_integrator_->delta.sqrt_inv_cov * r;

    return true;
  }

 private:
  const ImuState &imu_state_i_;
  const ImuState &imu_state_j_;
  std::shared_ptr<bs_common::PreIntegrator> pre_integrator_;
};