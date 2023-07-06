#include <bs_constraints/inertial/imu_state_3d_stamped_transaction.h>

#include <bs_constraints/inertial/absolute_imu_state_3d_stamped_constraint.h>
#include <bs_constraints/inertial/relative_imu_state_3d_stamped_constraint.h>

namespace bs_constraints { namespace inertial {

using ConstraintType =
    bs_constraints::inertial::RelativeImuState3DStampedConstraint;
using PriorType = bs_constraints::inertial::AbsoluteImuState3DStampedConstraint;

ImuState3DStampedTransaction::ImuState3DStampedTransaction(
    const ros::Time& transaction_stamp) {
  transaction_ = fuse_core::Transaction::make_shared();
  transaction_->stamp(transaction_stamp);
}

fuse_core::Transaction::SharedPtr
    ImuState3DStampedTransaction::GetTransaction() const {
  if (transaction_->empty()) { return nullptr; }
  return transaction_;
}

void ImuState3DStampedTransaction::AddPriorImuStateConstraint(
    const bs_common::ImuState& imu_state,
    const Eigen::Matrix<double, 15, 15>& prior_covariance,
    const std::string& prior_source) {
  // populate mean
  Eigen::Matrix<double, 16, 1> mean;
  mean << imu_state.OrientationQuat().w(), imu_state.OrientationQuat().x(),
      imu_state.OrientationQuat().y(), imu_state.OrientationQuat().z(),
      imu_state.PositionVec(), imu_state.VelocityVec(), imu_state.GyroBiasVec(),
      imu_state.AccelBiasVec();

  // build and add constraint
  auto prior = std::make_shared<PriorType>(prior_source, imu_state, mean,
                                           prior_covariance);
  transaction_->addConstraint(prior, true);
}

void ImuState3DStampedTransaction::AddRelativeImuStateConstraint(
    const bs_common::ImuState& imu_state_i,
    const bs_common::ImuState& imu_state_j,
    const bs_common::PreIntegrator& pre_integrator, const std::string& source) {
  std::shared_ptr<bs_common::PreIntegrator> pre_integrator_ptr =
      std::make_shared<bs_common::PreIntegrator>(pre_integrator);
  // build and add constraint
  auto constraint = ConstraintType::make_shared(
      source, imu_state_i, imu_state_j, pre_integrator_ptr);
  transaction_->addConstraint(constraint, true);
}

void ImuState3DStampedTransaction::AddImuStateVariables(
    const bs_common::ImuState& imu_state) {
  // extract fuse/beam variables
  const fuse_variables::Orientation3DStamped& orr = imu_state.Orientation();
  const fuse_variables::Position3DStamped& pos = imu_state.Position();
  const fuse_variables::VelocityLinear3DStamped& vel = imu_state.Velocity();
  const bs_variables::GyroscopeBias3DStamped& bg = imu_state.GyroBias();
  const bs_variables::AccelerationBias3DStamped& ba = imu_state.AccelBias();

  // add to transaction
  transaction_->addInvolvedStamp(imu_state.Stamp());
  // we do not want to override the pose
  transaction_->addVariable(
      fuse_variables::Orientation3DStamped::make_shared(orr), false);
  transaction_->addVariable(fuse_variables::Position3DStamped::make_shared(pos),
                            false);
  // we do want to override these
  transaction_->addVariable(
      fuse_variables::VelocityLinear3DStamped::make_shared(vel), true);
  transaction_->addVariable(
      bs_variables::GyroscopeBias3DStamped::make_shared(bg), true);
  transaction_->addVariable(
      bs_variables::AccelerationBias3DStamped::make_shared(ba), true);
}

}} // namespace bs_constraints::inertial