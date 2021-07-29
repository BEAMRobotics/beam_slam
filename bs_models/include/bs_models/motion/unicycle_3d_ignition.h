#pragma once

#include <atomic>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/macros.h>
#include <fuse_core/uuid.h>
#include <fuse_models/SetPose.h>
#include <fuse_models/SetPoseDeprecated.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <bs_parameters/models/unicycle_3d_ignition_params.h>

namespace bs_models { namespace motion {

/**
 * @brief A fuse_models ignition sensor designed to be used in conjunction with
 * the unicycle 3D motion model.
 *
 * TODO: Update description
 *
 */
class Unicycle3DIgnition : public fuse_core::AsyncSensorModel {
public:
  SMART_PTR_DEFINITIONS(Unicycle3DIgnition);
  using ParameterType = bs_parameters::models::Unicycle3DIgnitionParams;

  /**
   * @brief Default constructor
   *
   * All plugins are required to have a constructor that accepts no arguments
   */
  Unicycle3DIgnition();

  /**
   * @brief Destructor
   */
  ~Unicycle3DIgnition() = default;

  /**
   * @brief Subscribe to the input topic to start sending transactions to the
   * optimizer
   *
   * As a very special case, we are overriding the start() method instead of
   * providing an onStart() implementation. This is because the
   * Unicycle3DIgnition sensor calls reset() on the optimizer, which in turn
   * calls stop() and start(). If we used the AsyncSensorModel implementations
   * of start() and stop(), the system would hang inside of one callback
   * function while waiting for another callback to complete.
   */
  void start() override;

  /**
   * @brief Unsubscribe from the input topic to stop sending transactions to the
   * optimizer
   *
   * As a very special case, we are overriding the stop() method instead of
   * providing an onStop() implementation. This is because the
   * Unicycle3DIgnition sensor calls reset() on the optimizer, which in turn
   * calls stop() and start(). If we used the AsyncSensorModel implementations
   * of start() and stop(), the system would hang inside of one callback
   * function while waiting for another callback to complete.
   */
  void stop() override;

  /**
   * @brief Triggers the publication of a new prior transaction at the supplied
   * pose
   */
  void subscriberCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  /**
   * @brief Triggers the publication of a new prior transaction at the supplied
   * pose
   */
  bool setPoseServiceCallback(fuse_models::SetPose::Request& req,
                              fuse_models::SetPose::Response& res);

  /**
   * @brief Triggers the publication of a new prior transaction at the supplied
   * pose
   */
  bool setPoseDeprecatedServiceCallback(
      fuse_models::SetPoseDeprecated::Request& req,
      fuse_models::SetPoseDeprecated::Response&);

protected:
  /**
   * @brief Perform any required initialization for the kinematic ignition
   * sensor
   */
  void onInit() override;

  /**
   * @brief Process a received pose from one of the ROS comm channels
   *
   * This method validates the input pose, resets the optimizer, then constructs
   * and sends the initial state constraints (by calling sendPrior()).
   *
   * @param[in] pose - The pose and covariance to use for the prior constraints
   * on (x, y, yaw)
   */
  void process(const geometry_msgs::PoseWithCovarianceStamped& pose);

  /**
   * @brief Create and send a prior transaction based on the supplied pose
   *
   * The unicycle 2d state members not included in the pose message (x_vel,
   * y_vel, yaw_vel, x_acc, y_acc) will use the initial state values and
   * standard deviations configured on the parameter server.
   *
   * @param[in] pose - The pose and covariance to use for the prior constraints
   * on (x, y, yaw)
   */
  void sendPrior(const geometry_msgs::PoseWithCovarianceStamped& pose);

  std::atomic_bool started_; //!< Flag indicating the sensor has been started

  bool initial_transaction_sent_; //!< Flag indicating an initial transaction
                                  //!< has been sent already

  fuse_core::UUID device_id_; //!< The UUID of this device

  ParameterType
      params_; //!< Object containing all of the configuration parameters

  ros::ServiceClient reset_client_; //!< Service client used to call the "reset"
                                    //!< service on the optimizer

  ros::ServiceServer
      set_pose_service_; //!< ROS service server that receives SetPose requests

  ros::ServiceServer
      set_pose_deprecated_service_; //!< ROS service server that receives
                                    //!< SetPoseDeprecated requests

  ros::Subscriber subscriber_; //!< ROS subscriber that receives
                               //!< PoseWithCovarianceStamped messages
};

}} // namespace bs_models::motion
