#include <bs_models/inertial_odometry.h>

#include <beam_utils/utils.h>
#include <pluginlib/class_list_macros.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::InertialOdometry, fuse_core::SensorModel);

namespace bs_models {

using namespace vision;

InertialOdometry::InertialOdometry()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_imu_callback_(
          std::bind(&InertialOdometry::processIMU, this, std::placeholders::_1)),
      throttled_callback_(
          std::bind(&InertialOdometry::processOdometry, this, std::placeholders::_1))) {}

void InertialOdometry::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  calibration_params_.loadFromROS();
  params_.loadFromROS(private_node_handle_);

  // read imu parameters
  nlohmann::json J;
  beam::ReadJson(calibration_params_.imu_intrinsics_path, J);
  imu_params_.cov_prior_noise = J["cov_prior_noise"];
  imu_params_.cov_gyro_noise = Eigen::Matrix3d::Identity() * J["cov_gyro_noise"];
  imu_params_.cov_accel_noise = Eigen::Matrix3d::Identity() * J["cov_accel_noise"];
  imu_params_.cov_gyro_bias = Eigen::Matrix3d::Identity() * J["cov_gyro_bias"];
  imu_params_.cov_accel_bias = Eigen::Matrix3d::Identity() * J["cov_accel_bias"];
}

void InertialOdometry::onStart() {
  // subscribe to imu topic
  imu_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Imu>(
      ros::names::resolve(params_.imu_topic), 1000, &ThrottledIMUCallback::callback,
      &throttled_imu_callback_, ros::TransportHints().tcpNoDelay(false));

  // setup publishers
  relative_odom_publisher_ =
      private_node_handle_.advertise<CameraMeasurementMsg>("odom/relative", 100);
  world_odom_publisher_ = private_node_handle_.advertise<CameraMeasurementMsg>("odom/world", 100);
}

void InertialOdometry::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE("InertialOdometry received IMU measurements: " << msg->header.stamp);

  // push imu message onto buffer
  imu_buffer_.push(*msg);

  // return if its not initialized
  if (!initialized) return;

  const auto curr_msg = imu_buffer_.front();
  const auto curr_stamp = curr_msg.header.stamp;

  // process each imu message as it comes in
  imu_preint_->AddToBuffer(curr_msg);

  // get relative pose and publish
  const auto relative_pose = imu_preint_->GetRelativeMotion(prev_stamp_, curr_stamp);
  if (relative_pose.has_value()) {
    const auto [T_IMUprev_IMUcurr, cov] = relative_pose.value();
    T_ODOM_IMUprev_ = T_ODOM_IMUprev_ * T_IMUprev_IMUcurr;
    auto odom_msg = bs_common::TransformToOdometryMessage(
        curr_stamp, odom_seq, "Odom", extrinsics_->GetImuFrameId(), T_ODOM_IMUprev_, cov);
    relative_odom_publisher_.publish(odom_msg);
  }

  // get world pose and publish
  const auto world = imu_preint_->GetPose(curr_stamp);
  if (world_pose.has_value()) {
    const auto [T_WORLD_IMU, cov] = world_pose.value();
    auto odom_msg =
        bs_common::TransformToOdometryMessage(curr_stamp, odom_seq, extrinsics_->GetWorldFrameId(),
                                              extrinsics_->GetImuFrameId(), T_WORLD_IMU, cov);
    world_odom_publisher_.publish(odom_msg);
  }

  odom_seq++;
  prev_stamp_ = curr_stamp;
  imu_buffer_.pop();
}

void InertialOdometry::processOdometry(const nav_msgs::Odometry::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE("InertialOdometry received Odometry measurements: " << msg->header.stamp);

  // return if its not initialized
  if (!initialized) return;

  // add constraint at time
  auto transaction = imu_preint_->RegisterNewImuPreintegratedFactor(msg->header.stamp);
  sendTransaction(transaction);
}

void InertialOdometry::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  if (initialized) { imu_preint_->UpdateGraph(graph_msg); }

  /*****Perform initial setup******/
  ros::Time most_recent_stamp(0.0);
  for (auto& var : local_graph_->getVariables()) {
    fuse_variables::Position3DStamped::SharedPtr position =
        fuse_variables::Position3DStamped::make_shared();
    if (var.type() == position->type()) {
      *position = dynamic_cast<const fuse_variables::Position3DStamped&>(var);
    }
    if (position->stamp() > most_recent_stamp) { most_recent_stamp = position->stamp(); }
  }

  auto accel_bias = fuse_variables::AccelerationBias3DStamped::make_shared();
  auto ba_uuid =
      fuse_core::uuid::generate(accel_bias->type(), most_recent_stamp, fuse_core::uuid::NIL);
  *accel_bias =
      dynamic_cast<const fuse_variables::AccelerationBias3DStamped&> graph_msg->getVariable(
          ba_uuid);

  auto gyro_bias = fuse_variables::GyroscopeBias3DStamped::make_shared();
  auto bg_uuid =
      fuse_core::uuid::generate(gyro_bias->type(), most_recent_stamp, fuse_core::uuid::NIL);
  *gyro_bias =
      dynamic_cast<const fuse_variables::GyroscopeBias3DStamped&> graph_msg->getVariable(bg_uuid);

  auto velocity = fuse_variables::VelocityLinear3DStamped::make_shared();
  auto vel_uuid =
      fuse_core::uuid::generate(velocity->type(), most_recent_stamp, fuse_core::uuid::NIL);
  *velocity =
      dynamic_cast<const fuse_variables::VelocityLinear3DStamped&> graph_msg->getVariable(vel_uuid);

  auto position = fuse_variables::Position3DStamped::make_shared();
  auto pos_uuid =
      fuse_core::uuid::generate(position->type(), most_recent_stamp, fuse_core::uuid::NIL);
  *position =
      dynamic_cast<const fuse_variables::Position3DStamped&> graph_msg->getVariable(pos_uuid);

  auto orientation = fuse_variables::Orientation3DStamped::make_shared();
  auto or_uuid =
      fuse_core::uuid::generate(orientation->type(), most_recent_stamp, fuse_core::uuid::NIL);
  *orientation =
      dynamic_cast<const fuse_variables::Orientation3DStamped&> graph_msg->getVariable(or_uuid);

  // create imu preint object
  Eigen::Vector3d ba(accel_bias->x(), accel_bias->y(), accel_bias->z());
  Eigen::Vector3d bg(gyro_bias->x(), gyro_bias->y(), gyro_bias->z());
  imu_preint_ = std::make_shared<bs_models::ImuPreintegration>(imu_params_, bg, ba);

  // remove measurements before the start
  while (imu_buffer_.front().header.stamp <= most_recent_stamp && !imu_buffer_.empty()) {
    imu_buffer_.pop();
  }

  // set the start
  imu_preint_->SetStart(most_recent_stamp, orientation, position, velocity);

  // iterate through existing buffer, estimate poses and output
  prev_stamp_ = most_recent_stamp;
  while (!imu_buffer_.empty()) {
    auto imu_msg = imu_buffer_.front();
    imu_preint_->AddToBuffer(imu_msg);
    const auto curr_stamp = imu_msg.header.stamp;

    // get relative pose and publish
    const auto relative_pose = imu_preint_->GetRelativeMotion(prev_stamp, curr_stamp);
    if (relative_pose.has_value()) {
      const auto [T_IMUprev_IMUcurr, cov] = relative_pose.value();
      T_ODOM_IMUprev_ = T_ODOM_IMUprev_ * T_IMUprev_IMUcurr;
      auto odom_msg = bs_common::TransformToOdometryMessage(
          curr_stamp, odom_seq, "Odom", extrinsics_->GetImuFrameId(), T_ODOM_IMUprev_, cov);
      relative_odom_publisher_.publish(odom_msg);
    }

    // get world pose and publish
    const auto world = imu_preint_->GetPose(curr_stamp);
    if (world_pose.has_value()) {
      const auto [T_WORLD_IMU, cov] = world_pose.value();
      auto odom_msg = bs_common::TransformToOdometryMessage(
          curr_stamp, odom_seq, extrinsics_->GetWorldFrameId(), extrinsics_->GetImuFrameId(),
          T_WORLD_IMU, cov);
      world_odom_publisher_.publish(odom_msg);
    }

    odom_seq++;
    prev_stamp_ = curr_stamp;
    imu_buffer_.pop();
  }
}
