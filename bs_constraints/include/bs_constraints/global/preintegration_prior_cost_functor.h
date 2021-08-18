#pragma once

#include <ceres/ceres.h>

#include <bs_common/preintegrator.h>
#include <bs_common/imu_state.h>
#include <bs_constraints/frame_to_frame/preintegration_delta_cost_functor.h>

#include <beam_utils/math.h>

class PreIntegrationPriorCost
    : public ceres::SizedCostFunction<15, 4, 3, 3, 3, 3> {
 public:
  FUSE_MAKE_ALIGNED_OPERATOR_NEW();

  /**
   * @brief Constructor
   * @param imu_state_i current IMU state
   * @param imu_state_j new IMU state
   * @param pre_integrator preintegrator class containing IMU data between new
   * and current IMU states
   */
  PreIntegrationPriorCost(
      const bs_common::ImuState &imu_state_i,
      const bs_common::ImuState &imu_state_j,
      const std::shared_ptr<bs_common::PreIntegrator> pre_integrator)
      : error(imu_state_i, imu_state_j, pre_integrator),
        imu_state_i_(imu_state_i) {}

  /**
   * @brief Override for Cere's Evaluate function. See Cere's documentation for
   * parameter description
   */
  bool Evaluate(const double *const *parameters, double *residuals,
                double **jacobians) const override {
    std::array<const double *, 10> params = {imu_state_i_.Orientation().data(),
                                             imu_state_i_.Position().data(),
                                             imu_state_i_.Velocity().data(),
                                             imu_state_i_.GyroBias().data(),
                                             imu_state_i_.AccelBias().data(),
                                             parameters[0],
                                             parameters[1],
                                             parameters[2],
                                             parameters[3],
                                             parameters[4]};
    if (jacobians) {
      std::array<double *, 10> jacobs = {
          nullptr,      nullptr,      nullptr,      nullptr,      nullptr,
          jacobians[0], jacobians[1], jacobians[2], jacobians[3], jacobians[4]};
      return error.Evaluate(params.data(), residuals, jacobs.data());
    } else {
      return error.Evaluate(params.data(), residuals, nullptr);
    }
  }

 private:
  bs_constraints::frame_to_frame::PreIntegrationDeltaCost error;
  const bs_common::ImuState &imu_state_i_;
};
