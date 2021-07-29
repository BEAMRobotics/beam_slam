#include <bs_models/motion/unicycle_3d_ignition.h>

#include <Eigen/Eigenvalues>
#include <fuse_constraints/absolute_constraint.h>
#include <fuse_constraints/absolute_orientation_3d_stamped_constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/transaction.h>
#include <fuse_core/util.h>
#include <fuse_models/SetPose.h>
#include <fuse_models/SetPoseDeprecated.h>
#include <fuse_variables/acceleration_linear_3d_stamped.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/stamped.h>
#include <fuse_variables/velocity_angular_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>
#include <pluginlib/class_list_macros.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <bs_constraints/global/absolute_constraint.h>

// Register this motion model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::motion::Unicycle3DIgnition,
                       fuse_core::SensorModel);

namespace bs_models { namespace motion {

Unicycle3DIgnition::Unicycle3DIgnition()
    : fuse_core::AsyncSensorModel(1),
      started_(false),
      initial_transaction_sent_(false),
      device_id_(fuse_core::uuid::NIL) {}

void Unicycle3DIgnition::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);

  params_.loadFromROS(private_node_handle_);

  // Advertise
  subscriber_ = node_handle_.subscribe(
      ros::names::resolve(params_.topic), params_.queue_size,
      &Unicycle3DIgnition::subscriberCallback, this);
  set_pose_service_ = node_handle_.advertiseService(
      ros::names::resolve(params_.set_pose_service),
      &Unicycle3DIgnition::setPoseServiceCallback, this);
  set_pose_deprecated_service_ = node_handle_.advertiseService(
      ros::names::resolve(params_.set_pose_deprecated_service),
      &Unicycle3DIgnition::setPoseDeprecatedServiceCallback, this);
}

void Unicycle3DIgnition::start() {
  started_ = true;

  // TODO(swilliams) Should this be executed every time optimizer.reset() is
  // called, or only once ever?
  //                 I feel like it should be "only once ever".
  // Send an initial state transaction immediately, if requested
  if (params_.publish_on_startup && !initial_transaction_sent_) {
    auto pose = geometry_msgs::PoseWithCovarianceStamped();
    pose.header.stamp = ros::Time::now();
    pose.pose.pose.position.x = params_.initial_state[0];
    pose.pose.pose.position.y = params_.initial_state[1];
    pose.pose.pose.position.z = params_.initial_state[2];

    Eigen::Vector3d rpy = {params_.initial_state[6], params_.initial_state[7],
                           params_.initial_state[8]};
    Eigen::Affine3d orientation{
        Eigen::AngleAxis<double>(rpy[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxis<double>(rpy[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxis<double>(rpy[0], Eigen::Vector3d::UnitX())};
    Eigen::Quaterniond q(orientation.linear());

    pose.pose.pose.orientation =
        tf2::toMsg(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));

    //    [1, 0, 0, 0, 0, 0 - 0
    //     0, 1, 0, 0, 0, 0 - 7
    //     0, 0, 1, 0, 0, 0 - 14
    //     0, 0, 0, 1, 0, 0 - 21
    //     0, 0, 0, 0, 1, 0 - 28
    //     0, 0, 0, 0, 0, 1] -35
    pose.pose.covariance[0] =
        params_.initial_sigma[0] * params_.initial_sigma[0];
    pose.pose.covariance[7] =
        params_.initial_sigma[1] * params_.initial_sigma[1];
    pose.pose.covariance[14] =
        params_.initial_sigma[2] * params_.initial_sigma[2];
    pose.pose.covariance[21] =
        params_.initial_sigma[3] * params_.initial_sigma[3];
    pose.pose.covariance[28] =
        params_.initial_sigma[4] * params_.initial_sigma[4];
    pose.pose.covariance[35] =
        params_.initial_sigma[5] * params_.initial_sigma[5];

    sendPrior(pose);
    initial_transaction_sent_ = true;
  }
}

void Unicycle3DIgnition::stop() {
  started_ = false;
}

void Unicycle3DIgnition::subscriberCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  try {
    process(*msg);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what() << " Ignoring message.");
  }
}

bool Unicycle3DIgnition::setPoseServiceCallback(
    fuse_models::SetPose::Request& req, fuse_models::SetPose::Response& res) {
  try {
    process(req.pose);
    res.success = true;
  } catch (const std::exception& e) {
    res.success = false;
    res.message = e.what();
    ROS_ERROR_STREAM(e.what() << " Ignoring request.");
  }
  return true;
}

bool Unicycle3DIgnition::setPoseDeprecatedServiceCallback(
    fuse_models::SetPoseDeprecated::Request& req,
    fuse_models::SetPoseDeprecated::Response&) {
  try {
    process(req.pose);
    return true;
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what() << " Ignoring request.");
    return false;
  }
}

void Unicycle3DIgnition::process(
    const geometry_msgs::PoseWithCovarianceStamped& pose) {
  // Verify we are in the correct state to process set pose requests
  if (!started_) {
    throw std::runtime_error(
        "Attempting to set the pose while the sensor is stopped.");
  }
  // Validate the requested pose and covariance before we do anything
  if (!std::isfinite(pose.pose.pose.position.x) ||
      !std::isfinite(pose.pose.pose.position.y)) {
    throw std::invalid_argument(
        "Attempting to set the pose to an invalid position (" +
        std::to_string(pose.pose.pose.position.x) + ", " +
        std::to_string(pose.pose.pose.position.y) + ").");
  }
  auto orientation_norm =
      std::sqrt(pose.pose.pose.orientation.x * pose.pose.pose.orientation.x +
                pose.pose.pose.orientation.y * pose.pose.pose.orientation.y +
                pose.pose.pose.orientation.z * pose.pose.pose.orientation.z +
                pose.pose.pose.orientation.w * pose.pose.pose.orientation.w);
  if (std::abs(orientation_norm - 1.0) > 1.0e-3) {
    throw std::invalid_argument(
        "Attempting to set the pose to an invalid orientation (" +
        std::to_string(pose.pose.pose.orientation.x) + ", " +
        std::to_string(pose.pose.pose.orientation.y) + ", " +
        std::to_string(pose.pose.pose.orientation.z) + ", " +
        std::to_string(pose.pose.pose.orientation.w) + ").");
  }
  auto position_cov = fuse_core::Matrix2d();
  position_cov << pose.pose.covariance[0], pose.pose.covariance[1],
      pose.pose.covariance[6], pose.pose.covariance[7];
  if (!position_cov.isApprox(position_cov.transpose())) {
    throw std::invalid_argument("Attempting to set the pose with a "
                                "non-symmetric position covariance matrix [" +
                                std::to_string(position_cov(0)) + ", " +
                                std::to_string(position_cov(1)) + " ; " +
                                std::to_string(position_cov(2)) + ", " +
                                std::to_string(position_cov(3)) + "].");
  }
  Eigen::SelfAdjointEigenSolver<fuse_core::Matrix2d> solver(position_cov);
  if (solver.eigenvalues().minCoeff() <= 0.0) {
    throw std::invalid_argument(
        "Attempting to set the pose with a non-positive-definite position "
        "covariance matrix [" +
        std::to_string(position_cov(0)) + ", " +
        std::to_string(position_cov(1)) + " ; " +
        std::to_string(position_cov(2)) + ", " +
        std::to_string(position_cov(3)) + "].");
  }
  auto orientation_cov = fuse_core::Matrix1d();
  orientation_cov << pose.pose.covariance[35];
  if (orientation_cov(0) <= 0.0) {
    throw std::invalid_argument("Attempting to set the pose with a "
                                "non-positive-definite orientation covariance "
                                "matrix [" +
                                std::to_string(orientation_cov(0)) + "].");
  }

  // Now that the pose has been validated and the optimizer has been reset,
  // actually send the initial state constraints to the optimizer
  sendPrior(pose);
}

void Unicycle3DIgnition::sendPrior(
    const geometry_msgs::PoseWithCovarianceStamped& pose) {
  const auto& stamp = pose.header.stamp;

  // Create variables for the full state.
  // The initial values of the pose are extracted from the provided
  // PoseWithCovarianceStamped message. The remaining dimensions are provided as
  // parameters to the parameter server.
  auto position =
      fuse_variables::Position3DStamped::make_shared(stamp, device_id_);
  position->x() = pose.pose.pose.position.x;
  position->y() = pose.pose.pose.position.y;
  position->z() = pose.pose.pose.position.y;

  auto orientation =
      fuse_variables::Orientation3DStamped::make_shared(stamp, device_id_);
  orientation->x() = pose.pose.pose.orientation.x;
  orientation->y() = pose.pose.pose.orientation.y;
  orientation->z() = pose.pose.pose.orientation.z;
  orientation->w() = pose.pose.pose.orientation.w;

  auto linear_velocity =
      fuse_variables::VelocityLinear3DStamped::make_shared(stamp, device_id_);

  linear_velocity->x() = params_.initial_state[3];
  linear_velocity->y() = params_.initial_state[4];
  linear_velocity->z() = params_.initial_state[5];

  auto angular_velocity =
      fuse_variables::VelocityAngular3DStamped::make_shared(stamp, device_id_);
  angular_velocity->roll() = params_.initial_state[9];
  angular_velocity->pitch() = params_.initial_state[10];
  angular_velocity->yaw() = params_.initial_state[11];

  auto linear_acceleration =
      fuse_variables::AccelerationLinear3DStamped::make_shared(stamp,
                                                               device_id_);
  linear_acceleration->x() = params_.initial_state[12];
  linear_acceleration->y() = params_.initial_state[13];
  linear_acceleration->z() = params_.initial_state[14];

  auto position_cov = fuse_core::Matrix3d();
  position_cov << pose.pose.covariance[0], pose.pose.covariance[1],
      pose.pose.covariance[2], pose.pose.covariance[6], pose.pose.covariance[7],
      pose.pose.covariance[8], pose.pose.covariance[12],
      pose.pose.covariance[13], pose.pose.covariance[14];

  auto orientation_cov = fuse_core::Matrix3d();
  orientation_cov << pose.pose.covariance[21], pose.pose.covariance[22],
      pose.pose.covariance[23], pose.pose.covariance[27],
      pose.pose.covariance[28], pose.pose.covariance[29],
      pose.pose.covariance[33], pose.pose.covariance[34],
      pose.pose.covariance[35];

  auto linear_velocity_cov = fuse_core::Matrix3d();
  linear_velocity_cov << params_.initial_sigma[3] * params_.initial_sigma[3],
      0.0, 0.0, 0.0, params_.initial_sigma[4] * params_.initial_sigma[4], 0.0,
      0.0, 0.0, params_.initial_sigma[5] * params_.initial_sigma[5];

  auto angular_velocity_cov = fuse_core::Matrix3d();
  angular_velocity_cov << params_.initial_sigma[9] * params_.initial_sigma[9],
      0.0, 0.0, 0.0, params_.initial_sigma[10] * params_.initial_sigma[10], 0.0,
      0.0, 0.0, params_.initial_sigma[11] * params_.initial_sigma[11];

  auto linear_acceleration_cov = fuse_core::Matrix3d();
  linear_acceleration_cov << params_.initial_sigma[12] *
                                 params_.initial_sigma[12],
      0.0, 0.0, 0.0, params_.initial_sigma[13] * params_.initial_sigma[13], 0.0,
      0.0, 0.0, params_.initial_sigma[14] * params_.initial_sigma[14];

  // Create absolute constraints for each variable
  auto position_constraint =
      fuse_constraints::AbsolutePosition3DStampedConstraint::make_shared(
          name(), *position,
          fuse_core::Vector3d(position->x(), position->y(), position->z()),
          position_cov);

  auto orientation_constraint =
      fuse_constraints::AbsoluteOrientation3DStampedConstraint::make_shared(
          name(), *orientation,
          fuse_core::Vector4d(orientation->w(), orientation->x(),
                              orientation->y(), orientation->z()),
          orientation_cov);

  auto linear_velocity_constraint =
      bs_constraints::global::AbsoluteVelocityLinear3DStampedConstraint::make_shared(
          name(), *linear_velocity,
          fuse_core::Vector3d(linear_velocity->x(), linear_velocity->y(),
                              linear_velocity->z()),
          linear_velocity_cov);

  auto angular_velocity_constraint =
      bs_constraints::global::AbsoluteVelocityAngular3DStampedConstraint::make_shared(
          name(), *angular_velocity,
          fuse_core::Vector3d(angular_velocity->roll(),
                              angular_velocity->pitch(),
                              angular_velocity->yaw()),
          angular_velocity_cov);

  auto linear_acceleration_constraint =
      bs_constraints::global::AbsoluteAccelerationLinear3DStampedConstraint::
          make_shared(name(), *linear_acceleration,
                      fuse_core::Vector3d(linear_acceleration->x(),
                                          linear_acceleration->y(),
                                          linear_acceleration->z()),
                      linear_acceleration_cov);

  // Create the transaction
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(stamp);
  transaction->addInvolvedStamp(stamp);
  transaction->addVariable(position);
  transaction->addVariable(orientation);
  transaction->addVariable(linear_velocity);
  transaction->addVariable(angular_velocity);
  transaction->addVariable(linear_acceleration);
  transaction->addConstraint(position_constraint);
  transaction->addConstraint(orientation_constraint);
  transaction->addConstraint(linear_velocity_constraint);
  transaction->addConstraint(angular_velocity_constraint);
  transaction->addConstraint(linear_acceleration_constraint);

  // Send the transaction to the optimizer.
  sendTransaction(transaction);

  ROS_INFO_STREAM("Received a set_pose request (stamp: "
                  << stamp << ", x: " << position->x() << ", y: "
                  << position->y() << ", yaw: " << orientation->yaw() << ")");
}

}} // namespace bs_models::motion
