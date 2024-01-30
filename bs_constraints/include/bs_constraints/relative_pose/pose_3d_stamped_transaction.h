#pragma once

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_constraints/relative_pose_3d_stamped_constraint.h>
#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/transaction.h>
#include <fuse_loss/cauchy_loss.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

#include <bs_variables/orientation_3d.h>
#include <bs_variables/position_3d.h>

namespace bs_constraints {

/**
 * @brief this is mostly a helper class to adding relative pose constraints and
 * their associated variables to the graph. This ensures consistency between
 * everyone adding these types of constraints
 */
class Pose3DStampedTransaction {
public:
  FUSE_SMART_PTR_DEFINITIONS(Pose3DStampedTransaction);

  Pose3DStampedTransaction(const ros::Time& transaction_stamp,
                           bool override_constraints = true,
                           bool override_variables = true);

  /**
   * @brief get the transaction. If empty, it will return a nullptr
   */
  fuse_core::Transaction::SharedPtr GetTransaction() const;

  /**
   * @brief Add relative pose constraint. If frame_id is empty or equal to the
   * baselink frame, then the added constraint will be of type
   * fuse_constraints::AbsolutePose3DStampedConstraint. If the frame id is equal
   * to the camera frame id or lidar frame id, then the constraint will be of
   * type bs_constraints::RelativePose3DStampedWithExtrinsicsConstraint. If the
   * frame id is anything else, this will crash.
   * @param position1 position of frame 1 (baselink frame)
   * @param position2 position of frame 2 (baselink frame)
   * @param orientation1 orientation of frame 1 (baselink frame)
   * @param orientation2 orientation of frame 2 (baselink frame)
   * @param diff_Frame1_Frame2 measured difference between frame 2 and frame 1
   * measured as T_Frame1_Frame2 and stored as a vector [tx, ty, tz, qw, qx, qy,
   * qz]. This must be expressed in the same frame as the input frame_id. I.e.,
   * if frame_id = Lidar then this variable should measure: T_Lidar1_Lidar2
   * @param covariance covariance of the measurement
   * @param source source of the measurement
   * @param frame_id frame Id of the poses. See brief above for more info
   */
  void AddPoseConstraint(
      const fuse_variables::Position3DStamped& position1,
      const fuse_variables::Position3DStamped& position2,
      const fuse_variables::Orientation3DStamped& orientation1,
      const fuse_variables::Orientation3DStamped& orientation2,
      const Eigen::Matrix<double, 7, 1>& diff_Frame1_Frame2,
      const Eigen::Matrix<double, 6, 6>& covariance, const std::string& source,
      const std::string& frame_id = "");

  /**
   * @brief Add prior on a pose using a full covariance matrix.
   * NOTE: pose must be in baselink frame!
   */
  void AddPosePrior(const fuse_variables::Position3DStamped& position,
                    const fuse_variables::Orientation3DStamped& orientation,
                    const fuse_core::Matrix6d& prior_covariance,
                    const std::string& prior_source);

  /**
   * @brief Add prior on a pose using a diagonal value.
   * NOTE: pose must be in baselink frame!
   */
  void AddPosePrior(const fuse_variables::Position3DStamped& position,
                    const fuse_variables::Orientation3DStamped& orientation,
                    double prior_covariance_noise,
                    const std::string& prior_source);

  /**
   * @brief Add pose variables to the graph.
   * NOTE: pose must be in baselink frame!
   */
  void AddPoseVariables(const fuse_variables::Position3DStamped& position,
                        const fuse_variables::Orientation3DStamped& orientation,
                        const ros::Time& stamp);

  /**
   * @brief Add extrinsics variables to the graph for a specific frame id. This
   * will add the position and orientation as time invariant variables defining
   * a transform between frame_id and the baselink frame id
   */
  void AddExtrinsicVariablesForFrame(const std::string& frame_id,
                                     double extrinsics_prior = 0);

  void AddExtrinsicPrior(const bs_variables::Position3D& position,
                         const bs_variables::Orientation3D& orientation,
                         const fuse_core::Matrix6d& prior_covariance,
                         const std::string& prior_source,
                         bool override_prior = true);

protected:
  fuse_core::Transaction::SharedPtr transaction_;
  fuse_loss::CauchyLoss::SharedPtr loss_function_;
  bool override_constraints_;
  bool override_variables_;
};

} // namespace bs_constraints
