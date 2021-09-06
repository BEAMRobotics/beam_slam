#pragma once

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_constraints/relative_pose_3d_stamped_constraint.h>
#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/macros.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

#include <bs_common/imu_state.h>
#include <bs_common/utils.h>
#include <bs_constraints/relative_pose/relative_imu_state_3d_stamped_constraint.h>
#include <bs_constraints/global/absolute_imu_state_3d_stamped_constraint.h>

namespace bs_constraints {
namespace relative_pose {

/**
 *
 * This class is inherited by each of the different frame to frame (FTF)
 * transaction classes. The purpose of this class is two fold:
 *
 *  (1) to create some helper functions to reduce code and more clearly define
 * the variables (e.g., pose frame conventions)
 *
 *  (2) to enforce that FTF constraints are used in FTF sensor models. Standard
 * constraints from fuse_constraints can be defined below by simply defining an
 * alias with their specific template params (see
 * relative_pose_3d_stamped_transaction.h) with the option to define more
 * functions specific to their template types.
 *
 * NOTE: FTF constraint is one that constrains anything related to the state of
 * a frame. I.e., pose, velocity, acceleration (no keypoints or lidar points)
 *
 * NOTE: all frames described in these transactions must be the baselink frames
 *
 * @tparam ConstraintType template type for the FTF constraint being used. Note:
 * we have not implemented a FTF constraint base class to enforce this, we trust
 * that the derived FTF transaction classes will only use FTF constraints
 * @tparam PriorType template type for optional prior
 *
 */
template <typename ConstraintType, typename PriorType>
class RelativePoseTransactionBase {
 public:
  SMART_PTR_DEFINITIONS(RelativePoseTransactionBase<ConstraintType, PriorType>);

  RelativePoseTransactionBase(const ros::Time& transaction_stamp,
                              bool override_constraints = true,
                              bool override_variables = true)
      : override_constraints_(override_constraints),
        override_variables_(override_variables) {
    transaction_ = fuse_core::Transaction::make_shared();
    transaction_->stamp(transaction_stamp);
  }

  fuse_core::Transaction::SharedPtr GetTransaction() {
    if (transaction_->empty()) {
      return nullptr;
    }
    return transaction_;
  }

  void AddConstraint(const ConstraintType& constraint) {
    transaction_->addConstraint(constraint, override_constraints_);
  }

  void AddPoseConstraint(
      const Eigen::Matrix4d& T_WORLD_FRAME1,
      const Eigen::Matrix4d& T_WORLD_FRAME2, const ros::Time& stamp1,
      const ros::Time& stamp2, const Eigen::Matrix4d& T_FRAME1_FRAME2,
      const Eigen::Matrix<double, 6, 6>& covariance,
      const std::string& source = "NULL",
      const fuse_core::UUID& device_id = fuse_core::uuid::NIL) {
    // convert pose from Eigen to fuse
    auto p1 = fuse_variables::Position3DStamped::make_shared(stamp1, device_id);
    auto o1 =
        fuse_variables::Orientation3DStamped::make_shared(stamp1, device_id);
    bs_common::EigenTransformToFusePose(T_WORLD_FRAME1, *p1, *o1);

    auto p2 = fuse_variables::Position3DStamped::make_shared(stamp2, device_id);
    auto o2 =
        fuse_variables::Orientation3DStamped::make_shared(stamp2, device_id);
    bs_common::EigenTransformToFusePose(T_WORLD_FRAME2, *p2, *o2);

    // convert relative pose from Eigen to fuse
    Eigen::Matrix3d R = T_FRAME1_FRAME2.block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);
    fuse_core::Vector7d pose_relative_mean;
    pose_relative_mean << T_FRAME1_FRAME2(0, 3), T_FRAME1_FRAME2(1, 3),
        T_FRAME1_FRAME2(2, 3), q.w(), q.x(), q.y(), q.z();

    // build and add constraint
    auto constraint =
        fuse_constraints::RelativePose3DStampedConstraint::make_shared(
            source, *p1, *o1, *p2, *o2, pose_relative_mean, covariance);
    transaction_->addConstraint(constraint, override_constraints_);
  }

  void AddPoseConstraint(
      const fuse_variables::Position3DStamped& position1,
      const fuse_variables::Position3DStamped& position2,
      const fuse_variables::Orientation3DStamped& orientation1,
      const fuse_variables::Orientation3DStamped& orientation2,
      const fuse_variables::Position3DStamped& position2_relative,
      const fuse_variables::Orientation3DStamped& orientation2_relative,
      const Eigen::Matrix<double, 6, 6>& covariance,
      const std::string& source = "NULL") {
    // convert relative pose to vector
    fuse_core::Vector7d pose_relative_mean;
    pose_relative_mean << position2_relative.x(), position2_relative.y(),
        position2_relative.z(), orientation2_relative.w(),
        orientation2_relative.x(), orientation2_relative.y(),
        orientation2_relative.z();

    // build and add constraint
    auto constraint =
        fuse_constraints::RelativePose3DStampedConstraint::make_shared(
            source, position1, orientation1, position2, orientation2,
            pose_relative_mean, covariance);
    transaction_->addConstraint(constraint, override_constraints_);
  }

  void AddPrior(const PriorType& prior) {
    transaction_->addConstraint(prior, override_constraints_);
  }

  void AddPosePrior(const fuse_variables::Position3DStamped& position,
                    const fuse_variables::Orientation3DStamped& orientation,
                    const fuse_core::Matrix6d& prior_covariance,
                    const std::string& prior_source = "NULL") {
    fuse_core::Vector7d mean;
    mean << position.x(), position.y(), position.z(), orientation.w(),
        orientation.x(), orientation.y(), orientation.z();

    auto prior =
        std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
            prior_source, position, orientation, mean, prior_covariance);
    transaction_->addConstraint(prior, override_constraints_);
  }

  void AddPosePrior(const fuse_variables::Position3DStamped& position,
                    const fuse_variables::Orientation3DStamped& orientation,
                    double prior_covariance_noise,
                    const std::string& prior_source = "NULL") {
    fuse_core::Matrix6d prior_covariance_matrix{
        fuse_core::Matrix6d::Identity()};
    for (int i = 0; i < 6; i++) {
        prior_covariance_matrix(i, i) = prior_covariance_noise;
    }
    AddPosePrior(position, orientation, prior_covariance_matrix, prior_source);
  }

  void AddPoseVariables(
      const Eigen::Matrix4d& T_WORLD_FRAME, const ros::Time& stamp,
      const fuse_core::Matrix6d& prior_covariance,
      const std::string& prior_source = "NULL",
      const fuse_core::UUID& device_id = fuse_core::uuid::NIL) {
    transaction_->addInvolvedStamp(stamp);

    // create fuse variables
    auto p = fuse_variables::Position3DStamped::make_shared(stamp, device_id);
    auto o =
        fuse_variables::Orientation3DStamped::make_shared(stamp, device_id);
    bs_common::EigenTransformToFusePose(T_WORLD_FRAME, *p, *o);

    // add to transaction
    transaction_->addVariable(p, override_variables_);
    transaction_->addVariable(o, override_variables_);
    AddPosePrior(prior_source, *p, *o, prior_covariance);
  }

  void AddPoseVariables(
      const Eigen::Matrix4d& T_WORLD_FRAME, const ros::Time& stamp,
      const fuse_core::UUID& device_id = fuse_core::uuid::NIL) {
    transaction_->addInvolvedStamp(stamp);

    // create fuse variables
    auto p = fuse_variables::Position3DStamped::make_shared(stamp, device_id);
    auto o =
        fuse_variables::Orientation3DStamped::make_shared(stamp, device_id);
    bs_common::EigenTransformToFusePose(T_WORLD_FRAME, *p, *o);

    // add to transaction
    transaction_->addVariable(p, override_variables_);
    transaction_->addVariable(o, override_variables_);
  }

  void AddPoseVariables(const fuse_variables::Position3DStamped& position,
                        const fuse_variables::Orientation3DStamped& orientation,
                        const ros::Time& stamp) {
    transaction_->addInvolvedStamp(stamp);

    // add to transaction
    transaction_->addVariable(
        fuse_variables::Position3DStamped::make_shared(position),
        override_variables_);
    transaction_->addVariable(
        fuse_variables::Orientation3DStamped::make_shared(orientation),
        override_variables_);
  }

  void AddPriorImuStateConstraint(
      const bs_common::ImuState& imu_state,
      const Eigen::Matrix<double, 15, 15>& prior_covariance,
      const std::string& prior_source = "NULL") {
    // populate mean
    Eigen::Matrix<double, 16, 1> mean;
    mean << imu_state.OrientationQuat().w(), imu_state.OrientationQuat().x(),
        imu_state.OrientationQuat().y(), imu_state.OrientationQuat().z(),
        imu_state.PositionVec(), imu_state.VelocityVec(),
        imu_state.GyroBiasVec(), imu_state.AccelBiasVec();

    // build and add constraint
    auto prior = std::make_shared<
        bs_constraints::global::AbsoluteImuState3DStampedConstraint>(
        prior_source, imu_state, mean, prior_covariance);
    transaction_->addConstraint(prior, override_constraints_);
  }

  void AddRelativeImuStateConstraint(
      const bs_common::ImuState& imu_state_i,
      const bs_common::ImuState& imu_state_j,
      const bs_common::PreIntegrator& pre_integrator,
      const std::string& source = "NULL") {
    // build and add constraint
    auto constraint =
        bs_constraints::relative_pose::RelativeImuState3DStampedConstraint::
            make_shared(source, imu_state_i, imu_state_j, pre_integrator);
    transaction_->addConstraint(constraint, override_constraints_);
  }

  void AddImuStateVariables(const bs_common::ImuState& imu_state) {
    // extract fuse/beam variables
    const fuse_variables::Orientation3DStamped& orr = imu_state.Orientation();
    const fuse_variables::Position3DStamped& pos = imu_state.Position();
    const fuse_variables::VelocityLinear3DStamped& vel = imu_state.Velocity();
    const bs_variables::GyroscopeBias3DStamped& bg = imu_state.GyroBias();
    const bs_variables::AccelerationBias3DStamped& ba = imu_state.AccelBias();

    // add to transaction
    transaction_->addInvolvedStamp(imu_state.Stamp());
    transaction_->addVariable(
        fuse_variables::Orientation3DStamped::make_shared(orr),
        override_variables_);
    transaction_->addVariable(
        fuse_variables::Position3DStamped::make_shared(pos),
        override_variables_);
    transaction_->addVariable(
        fuse_variables::VelocityLinear3DStamped::make_shared(vel),
        override_variables_);
    transaction_->addVariable(
        bs_variables::GyroscopeBias3DStamped::make_shared(bg),
        override_variables_);
    transaction_->addVariable(
        bs_variables::AccelerationBias3DStamped::make_shared(ba),
        override_variables_);
  }

 protected:
  fuse_core::Transaction::SharedPtr transaction_;
  bool override_constraints_;
  bool override_variables_;
};

}  // namespace relative_pose
}  // namespace bs_constraints
