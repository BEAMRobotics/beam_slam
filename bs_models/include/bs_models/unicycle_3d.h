#pragma once

#include <map>

#include <fuse_core/async_motion_model.h>
#include <fuse_core/constraint.h>
#include <fuse_core/graph.h>
#include <fuse_core/macros.h>
#include <fuse_core/timestamp_manager.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <ros/ros.h>
#include <tf2/utils.h>

namespace bs_models {

class Unicycle3D : public fuse_core::AsyncMotionModel {
 public:
  SMART_PTR_DEFINITIONS_WITH_EIGEN(Unicycle3D);

  /**
   * @brief Default constructor
   *
   * All plugins are required to have a constructor that accepts no arguments
   */
  Unicycle3D();

  /**
   * @brief Destructor
   */
  ~Unicycle3D() = default;

 protected:
  /**
   * @brief Structure used to maintain a history of "good" pose estimates
   */
  struct StateHistoryElement {
    fuse_core::UUID
        position_uuid;  //!< The uuid of the associated position variable
    fuse_core::UUID
        orientation_uuid;  //!< The uuid of the associated orientation variable
    fuse_core::UUID
        vel_linear_uuid;  //!< The uuid of the associated orientation variable
    fuse_core::UUID
        vel_angular_uuid;  //!< The uuid of the associated orientation variable
    fuse_core::UUID
        acc_linear_uuid;  //!< The uuid of the associated orientation variable
    tf2::Transform pose;
    tf2::Vector3 velocity_linear;
    tf2::Vector3 velocity_angular;
    tf2::Vector3 acceleration_linear;
  };
  using StateHistory = std::map<ros::Time, StateHistoryElement>;
  /**
   * @brief Augment a transaction structure such that the provided timestamps
   * are connected by motion model constraints.
   * @param[in]  stamps      The set of timestamps that should be connected by
   * motion model constraints
   * @param[out] transaction The transaction object that should be augmented
   * with motion model constraints
   * @return                 True if the motion models were generated
   * successfully, false otherwise
   */
  bool applyCallback(fuse_core::Transaction& transaction) override;

  /**
   * @brief Generate a single motion model segment between the specified
   * timestamps.
   *
   * This function is used by the timestamp manager to generate just the new
   * motion model segments required to fulfill a query.
   *
   * @param[in]  beginning_stamp The beginning timestamp of the motion model
   * constraints to be generated. \p beginning_stamp is guaranteed to be less
   * than \p ending_stamp.
   * @param[in]  ending_stamp    The ending timestamp of the motion model
   * constraints to be generated. \p ending_stamp is guaranteed to be greater
   * than \p beginning_stamp.
   * @param[out] constraints     One or more motion model constraints between
   * the requested timestamps.
   * @param[out] variables       One or more variables at both the \p
   * beginning_stamp and \p ending_stamp. The variables should include initial
   * values for the optimizer.
   */
  void generateMotionModel(
      const ros::Time& beginning_stamp, const ros::Time& ending_stamp,
      std::vector<fuse_core::Constraint::SharedPtr>& constraints,
      std::vector<fuse_core::Variable::SharedPtr>& variables);

  /**
   * @brief Callback fired in the local callback queue thread(s) whenever a new
   * Graph is received from the optimizer
   * @param[in] graph A read-only pointer to the graph object, allowing queries
   * to be performed whenever needed.
   */
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) override;

  /**
   * @brief Perform any required initialization for the kinematic model
   */
  void onInit() override;

  /**
   * @brief Reset the internal state history before starting
   */
  void onStart() override;

  /**
   * @brief Update all of the estimated states in the state history container
   * using the optimized values from the graph
   * @param[in] graph         The graph object containing updated variable
   * values
   * @param[in] state_history The state history object to be updated
   * @param[in] buffer_length States older than this in the history will be
   * pruned
   */
  static void updateStateHistoryEstimates(const fuse_core::Graph& graph,
                                          StateHistory& state_history,
                                          const ros::Duration& buffer_length);

  ros::Duration buffer_length_;  //!< The length of the state history
  fuse_core::UUID device_id_;    //!< The UUID of the device to be published
  fuse_core::TimestampManager
      timestamp_manager_;  //!< Tracks timestamps and previously created motion
                           //!< model segments
  Eigen::Matrix<double, 15, 15, Eigen::RowMajor>
      process_noise_covariance_;  //!< Process noise covariance matrix

  StateHistory state_history_;  //!< History of optimized graph pose estimates
};

}  // namespace bs_models
