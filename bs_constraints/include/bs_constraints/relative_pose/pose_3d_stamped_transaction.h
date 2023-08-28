#pragma once

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_constraints/relative_pose_3d_stamped_constraint.h>
#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/transaction.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

namespace bs_constraints {

class Pose3DStampedTransaction {
public:
  FUSE_SMART_PTR_DEFINITIONS(Pose3DStampedTransaction);

  Pose3DStampedTransaction(const ros::Time& transaction_stamp,
                           bool override_constraints = true,
                           bool override_variables = true);

  fuse_core::Transaction::SharedPtr GetTransaction() const;

  void AddPoseConstraint(
      const fuse_variables::Position3DStamped& position1,
      const fuse_variables::Position3DStamped& position2,
      const fuse_variables::Orientation3DStamped& orientation1,
      const fuse_variables::Orientation3DStamped& orientation2,
      const fuse_variables::Position3DStamped& position2_relative,
      const fuse_variables::Orientation3DStamped& orientation2_relative,
      const Eigen::Matrix<double, 6, 6>& covariance,
      const std::string& source = "NULL");

  void AddPosePrior(const fuse_variables::Position3DStamped& position,
                    const fuse_variables::Orientation3DStamped& orientation,
                    const fuse_core::Matrix6d& prior_covariance,
                    const std::string& prior_source = "NULL");

  void AddPosePrior(const fuse_variables::Position3DStamped& position,
                    const fuse_variables::Orientation3DStamped& orientation,
                    double prior_covariance_noise,
                    const std::string& prior_source = "NULL");

  void AddPoseVariables(const fuse_variables::Position3DStamped& position,
                        const fuse_variables::Orientation3DStamped& orientation,
                        const ros::Time& stamp);

protected:
  fuse_core::Transaction::SharedPtr transaction_;
  bool override_constraints_;
  bool override_variables_;
};

} // namespace bs_constraints
