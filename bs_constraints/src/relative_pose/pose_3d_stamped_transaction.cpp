#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>

#include <bs_common/conversions.h>
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_constraints/relative_pose/relative_pose_3d_stamped_with_extrinsics_constraint.h>
#include <bs_variables/orientation_3d.h>
#include <bs_variables/position_3d.h>

namespace bs_constraints {

Pose3DStampedTransaction::Pose3DStampedTransaction(
    const ros::Time& transaction_stamp, bool override_constraints,
    bool override_variables)
    : override_constraints_(override_constraints),
      override_variables_(override_variables) {
  transaction_ = fuse_core::Transaction::make_shared();
  transaction_->stamp(transaction_stamp);
}

fuse_core::Transaction::SharedPtr
    Pose3DStampedTransaction::GetTransaction() const {
  if (transaction_->empty()) { return nullptr; }
  return transaction_;
}

void Pose3DStampedTransaction::AddPoseConstraint(
    const fuse_variables::Position3DStamped& position1,
    const fuse_variables::Position3DStamped& position2,
    const fuse_variables::Orientation3DStamped& orientation1,
    const fuse_variables::Orientation3DStamped& orientation2,
    const Eigen::Matrix<double, 7, 1>& diff_Frame1_Frame2,
    const Eigen::Matrix<double, 6, 6>& covariance, const std::string& source,
    const std::string& frame_id) {
  // add regular relative pose constraint if no frame id is provided
  if (frame_id.empty()) {
    auto constraint =
        fuse_constraints::RelativePose3DStampedConstraint::make_shared(
            source, position1, orientation1, position2, orientation2,
            diff_Frame1_Frame2, covariance);
    transaction_->addConstraint(constraint, override_constraints_);
    return;
  }

  bs_common::ExtrinsicsLookupOnline& extrinsics =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  // add regular relative pose constraint if frame id provided is the baselink
  // frame id
  if (frame_id == extrinsics.GetBaselinkFrameId()) {
    auto constraint =
        fuse_constraints::RelativePose3DStampedConstraint::make_shared(
            source, position1, orientation1, position2, orientation2,
            diff_Frame1_Frame2, covariance);
    transaction_->addConstraint(constraint, override_constraints_);
    return;
  }

  if (frame_id != extrinsics.GetLidarFrameId() &&
      frame_id != extrinsics.GetCameraFrameId()) {
    BEAM_ERROR(
        "Invalid frame Id: {}, not adding extrinsics variables to the graph",
        frame_id);
    return;
  }

  // add pose constraint with extrinsics
  bs_variables::Position3D p_extrinsics(extrinsics.GetBaselinkFrameId(),
                                        frame_id);
  bs_variables::Orientation3D o_extrinsics(extrinsics.GetBaselinkFrameId(),
                                           frame_id);
  auto constraint =
      bs_constraints::RelativePose3DStampedWithExtrinsicsConstraint::
          make_shared(source, position1, orientation1, position2, orientation2,
                      p_extrinsics, o_extrinsics, diff_Frame1_Frame2,
                      covariance);
  transaction_->addConstraint(constraint, override_constraints_);
}

void Pose3DStampedTransaction::AddPosePrior(
    const fuse_variables::Position3DStamped& position,
    const fuse_variables::Orientation3DStamped& orientation,
    const fuse_core::Matrix6d& prior_covariance,
    const std::string& prior_source) {
  fuse_core::Vector7d mean;
  mean << position.x(), position.y(), position.z(), orientation.w(),
      orientation.x(), orientation.y(), orientation.z();

  auto prior =
      std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
          prior_source, position, orientation, mean, prior_covariance);
  transaction_->addConstraint(prior, override_constraints_);
}

void Pose3DStampedTransaction::AddPosePrior(
    const fuse_variables::Position3DStamped& position,
    const fuse_variables::Orientation3DStamped& orientation,
    double prior_covariance_noise, const std::string& prior_source) {
  fuse_core::Matrix6d prior_covariance_matrix{fuse_core::Matrix6d::Identity()};
  for (int i = 0; i < 6; i++) {
    prior_covariance_matrix(i, i) = prior_covariance_noise;
  }
  AddPosePrior(position, orientation, prior_covariance_matrix, prior_source);
}

void Pose3DStampedTransaction::AddPoseVariables(
    const fuse_variables::Position3DStamped& position,
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

void Pose3DStampedTransaction::AddExtrinsicVariablesForFrame(
    const std::string& frame_id) {
  bs_common::ExtrinsicsLookupOnline& extrinsics =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  if (frame_id == extrinsics.GetBaselinkFrameId()) {
    BEAM_WARN("Cannot add extrinsics calibration for baselink frame");
    return;
  }

  if (frame_id != extrinsics.GetLidarFrameId() &&
      frame_id != extrinsics.GetCameraFrameId()) {
    BEAM_ERROR(
        "Invalid frame Id: {}, not adding extrinsics variables to the graph",
        frame_id);
    return;
  }

  Eigen::Matrix4d T_Baselink_Lidar;
  if (!extrinsics.GetT_BASELINK_LIDAR(T_Baselink_Lidar, ros::Time::now())) {
    BEAM_ERROR("Cannot get Lidar extrinsics");
    throw std::runtime_error{"extrinsics lookup error"};
  }
  Eigen::Matrix3d R = T_Baselink_Lidar.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);

  auto p = bs_variables::Position3D::make_shared(
      extrinsics.GetBaselinkFrameId(), frame_id);
  auto o = bs_variables::Orientation3D::make_shared(
      extrinsics.GetBaselinkFrameId(), frame_id);

  p->x() = T_Baselink_Lidar(0, 3);
  p->y() = T_Baselink_Lidar(1, 3);
  p->z() = T_Baselink_Lidar(2, 3);

  o->w() = q.w();
  o->x() = q.x();
  o->y() = q.y();
  o->z() = q.z();

  transaction_->addVariable(p, override_variables_);
  transaction_->addVariable(o, override_variables_);
}

} // namespace bs_constraints