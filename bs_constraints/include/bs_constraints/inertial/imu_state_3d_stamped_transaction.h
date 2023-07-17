#pragma once

#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>

#include <bs_common/imu_state.h>
#include <bs_common/preintegrator.h>
#include <bs_common/utils.h>

namespace bs_constraints { namespace inertial {

/// @brief
class ImuState3DStampedTransaction {
public:
  FUSE_SMART_PTR_DEFINITIONS(ImuState3DStampedTransaction);

  /// @brief
  /// @param transaction_stamp
  ImuState3DStampedTransaction(const ros::Time& transaction_stamp);

  /// @brief
  /// @return
  fuse_core::Transaction::SharedPtr GetTransaction() const;

  /// @brief
  /// @param imu_state
  /// @param prior_covariance
  /// @param prior_source
  void AddPriorImuStateConstraint(
      const bs_common::ImuState& imu_state,
      const Eigen::Matrix<double, 15, 15>& prior_covariance,
      const std::string& prior_source = "NULL");

  /// @brief
  /// @param imu_state_i
  /// @param imu_state_j
  /// @param pre_integrator
  /// @param source
  void AddRelativeImuStateConstraint(
      const bs_common::ImuState& imu_state_i,
      const bs_common::ImuState& imu_state_j,
      const bs_common::PreIntegrator& pre_integrator, const double info_weight,
      const std::string& source = "NULL");

  /// @brief
  /// @param imu_state
  void AddImuStateVariables(const bs_common::ImuState& imu_state);

protected:
  fuse_core::Transaction::SharedPtr transaction_;
};

}} // namespace bs_constraints::inertial