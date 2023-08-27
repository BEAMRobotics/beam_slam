#pragma once

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_constraints/relative_pose_3d_stamped_constraint.h>
#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/transaction.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

#include <bs_variables/orientation_3d.h>
#include <bs_variables/position_3d.h>

namespace bs_constraints {

class Pose3DStampedTransaction {
public:
  FUSE_SMART_PTR_DEFINITIONS(Pose3DStampedTransaction);

  Pose3DStampedTransaction(const ros::Time& transaction_stamp,
                           bool override_constraints = true,
                           bool override_variables = true)
      : override_constraints_(override_constraints),
        override_variables_(override_variables) {
    transaction_ = fuse_core::Transaction::make_shared();
    transaction_->stamp(transaction_stamp);
  }

  fuse_core::Transaction::SharedPtr GetTransaction() const {
    if (transaction_->empty()) { return nullptr; }
    return transaction_;
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

protected:
  fuse_core::Transaction::SharedPtr transaction_;
  bool override_constraints_;
  bool override_variables_;
};

} // namespace bs_constraints
