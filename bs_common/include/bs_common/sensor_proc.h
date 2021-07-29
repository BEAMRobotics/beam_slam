#pragma once

#include <fuse_constraints/relative_pose_3d_stamped_constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <beam_utils/math.h>

namespace bs_common {

/**
 * @brief Extracts relative 3D pose data from a PoseWithCovarianceStamped and
 * adds that data to a fuse Transaction
 *
 * This method computes the delta between two poses and creates the required
 * fuse variables and constraints, and then adds them to the given \p
 * transaction. The pose delta is calculated as:
 *
 * pose_relative = pose_absolute1^-1 * pose_absolute2
 *
 * Additionally, the covariance of each pose message is rotated into the robot's
 * base frame at the time of pose_absolute1. They are then added in the
 * constraint. This assumes independence between the pose measurements.
 *
 * @param[in] source - The name of the sensor or motion model that generated
 * this constraint
 * @param[in] device_id - The UUID of the machine
 * @param[in] pose1 - The first (and temporally earlier)
 * PoseWithCovarianceStamped message
 * @param[in] pose2 - The first (and temporally later) PoseWithCovarianceStamped
 * message
 * @param[out] transaction - The generated variables and constraints are added
 * to this transaction
 * @return true if any constraints were added, false otherwise
 */
inline bool processRelativePoseWithCovariance(
    const std::string& source, const fuse_core::UUID& device_id,
    const geometry_msgs::PoseWithCovarianceStamped& pose1,
    const geometry_msgs::PoseWithCovarianceStamped& pose2,
    fuse_core::Transaction& transaction) {
  // Convert the poses into tf2 transforms
  tf2::Transform absolute_pose1_3d;
  tf2::fromMsg(pose1.pose.pose, absolute_pose1_3d);

  tf2::Transform absolute_pose2_3d;
  tf2::fromMsg(pose2.pose.pose, absolute_pose2_3d);

  // Create the pose variables
  auto position1 = fuse_variables::Position3DStamped::make_shared(
      pose1.header.stamp, device_id);
  auto orientation1 = fuse_variables::Orientation3DStamped::make_shared(
      pose1.header.stamp, device_id);
  position1->x() = absolute_pose1_3d.getOrigin().x();
  position1->y() = absolute_pose1_3d.getOrigin().y();
  position1->z() = absolute_pose1_3d.getOrigin().z();
  orientation1->x() = absolute_pose1_3d.getRotation().x();
  orientation1->y() = absolute_pose1_3d.getRotation().y();
  orientation1->z() = absolute_pose1_3d.getRotation().z();
  orientation1->w() = absolute_pose1_3d.getRotation().w();

  auto position2 = fuse_variables::Position3DStamped::make_shared(
      pose2.header.stamp, device_id);
  auto orientation2 = fuse_variables::Orientation3DStamped::make_shared(
      pose2.header.stamp, device_id);
  position2->x() = absolute_pose2_3d.getOrigin().x();
  position2->y() = absolute_pose2_3d.getOrigin().y();
  position2->z() = absolute_pose2_3d.getOrigin().z();
  orientation2->x() = absolute_pose2_3d.getRotation().x();
  orientation2->y() = absolute_pose2_3d.getRotation().y();
  orientation2->z() = absolute_pose2_3d.getRotation().z();
  orientation2->w() = absolute_pose2_3d.getRotation().w();

  // Transform into eigen affines for calculating delta
  Eigen::Affine3d pose1_affine;
  pose1_affine.translation() =
      Eigen::Vector3d{position1->x(), position1->y(), position1->z()};
  pose1_affine.linear() =
      Eigen::Matrix3d{Eigen::Quaterniond{orientation1->w(), orientation1->x(),
                                         orientation1->y(), orientation1->z()}};

  Eigen::Affine3d pose2_affine;
  pose2_affine.translation() =
      Eigen::Vector3d{position2->x(), position2->y(), position2->z()};
  pose2_affine.linear() =
      Eigen::Matrix3d{Eigen::Quaterniond{orientation2->w(), orientation2->x(),
                                         orientation2->y(), orientation2->z()}};

  Eigen::Affine3d affine_delta = pose1_affine.inverse() * pose2_affine;

  Eigen::Quaterniond q(affine_delta.linear());

  Eigen::Vector3d rpy = affine_delta.linear().eulerAngles(2, 1, 0);

  // Create the delta for the constraint
  fuse_core::Vector7d pose_relative_mean;
  pose_relative_mean << affine_delta.translation().x(),
      affine_delta.translation().y(), affine_delta.translation().z(), q.w(),
      q.x(), q.y(), q.z();

  fuse_core::Matrix6d pose_relative_covariance;
  for (int i = 0; i < pose_relative_covariance.rows(); ++i) {
    for (int j = 0; j < pose_relative_covariance.cols(); ++j) {
      pose_relative_covariance(i, j) =
          pose1.pose.covariance[i * pose_relative_covariance.rows() + j];
    }
  }

  // Create a relative pose constraint. We assume the pose measurements are
  // independent.
  auto constraint =
      fuse_constraints::RelativePose3DStampedConstraint::make_shared(
          source, *position1, *orientation1, *position2, *orientation2,
          pose_relative_mean, pose_relative_covariance);

  transaction.addVariable(position1);
  transaction.addVariable(orientation1);
  transaction.addVariable(position2);
  transaction.addVariable(orientation2);
  transaction.addConstraint(constraint, true);
  transaction.addInvolvedStamp(pose1.header.stamp);
  transaction.addInvolvedStamp(pose2.header.stamp);

  return true;
}

inline bool processRelativePoseWithCovariance(
    const std::string& source,
    const fuse_variables::Position3DStamped::SharedPtr& position1,
    const fuse_variables::Orientation3DStamped::SharedPtr& orientation1,
    const fuse_variables::Position3DStamped::SharedPtr& position2,
    const fuse_variables::Orientation3DStamped::SharedPtr& orientation2,
    const Eigen::Matrix4d& T_CLOUD1_CLOUD2,
    const Eigen::Matrix<double, 6, 6>& covariance,
    fuse_core::Transaction& transaction,
    bool add_pose1_variable,
    bool add_pose2_variable) {
  // convert rotation matrix to quaternion
  Eigen::Matrix3d R = T_CLOUD1_CLOUD2.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);

  // Convert measurement to fuse variable
  fuse_core::Vector7d pose_relative_mean;
  pose_relative_mean << T_CLOUD1_CLOUD2(0, 3), T_CLOUD1_CLOUD2(1, 3),
      T_CLOUD1_CLOUD2(2, 3), q.w(), q.x(), q.y(), q.z();

  // Create a relative pose constraint. We assume the pose measurements are
  // independent.
  auto constraint =
      fuse_constraints::RelativePose3DStampedConstraint::make_shared(
          source, *position1, *orientation1, *position2, *orientation2,
          pose_relative_mean, covariance);
  if(add_pose1_variable){
    transaction.addVariable(position1, true);
    transaction.addVariable(orientation1, true);
  }    
  if(add_pose2_variable){
    transaction.addVariable(position2, true);
    transaction.addVariable(orientation2, true);
  }
  transaction.addConstraint(constraint, true);
  transaction.addInvolvedStamp(position1->stamp());
  transaction.addInvolvedStamp(position2->stamp());
  
  return true;
}

} // namespace bs_common
