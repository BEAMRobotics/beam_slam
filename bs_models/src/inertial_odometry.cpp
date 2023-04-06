#include <bs_models/inertial_odometry.h>

#include <beam_utils/utils.h>
#include <pluginlib/class_list_macros.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::InertialOdometry, fuse_core::SensorModel);

namespace bs_models {

InertialOdometry::InertialOdometry()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_imu_callback_(std::bind(&InertialOdometry::processIMU, this,
                                        std::placeholders::_1)),
      throttled_odom_callback_(std::bind(&InertialOdometry::processOdometry,
                                         this, std::placeholders::_1)) {}

void InertialOdometry::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  calibration_params_.loadFromROS();
  params_.loadFromROS(private_node_handle_);

  // check for non empty odom topic
  if (params_.constraint_odom_topic.empty()) {
    ROS_ERROR("Constraint odom topic cannot be empty, must be a valid odometry "
              "topic.");
    throw std::invalid_argument{"Constraint odom topic cannot be empty, must "
                                "be a valid odometry topic."};
  }

  // read imu parameters
  nlohmann::json J;
  beam::ReadJson(calibration_params_.imu_intrinsics_path, J);
  imu_params_.cov_prior_noise = J["cov_prior_noise"];
  imu_params_.cov_gyro_noise =
      Eigen::Matrix3d::Identity() * J["cov_gyro_noise"];
  imu_params_.cov_accel_noise =
      Eigen::Matrix3d::Identity() * J["cov_accel_noise"];
  imu_params_.cov_gyro_bias = Eigen::Matrix3d::Identity() * J["cov_gyro_bias"];
  imu_params_.cov_accel_bias =
      Eigen::Matrix3d::Identity() * J["cov_accel_bias"];
}

void InertialOdometry::onStart() {
  // subscribe to imu topic
  imu_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Imu>(
      ros::names::resolve(params_.imu_topic), 1000,
      &ThrottledIMUCallback::callback, &throttled_imu_callback_,
      ros::TransportHints().tcpNoDelay(false));

  if (!params_.constraint_odom_topic.empty()) {
    odom_subscriber_ = private_node_handle_.subscribe<nav_msgs::Odometry>(
        ros::names::resolve(params_.constraint_odom_topic), 1000,
        &ThrottledOdomCallback::callback, &throttled_odom_callback_,
        ros::TransportHints().tcpNoDelay(false));
  }

  // setup publishers
  relative_odom_publisher_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odom/relative", 100);
  world_odom_publisher_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odom/world", 100);
}

void InertialOdometry::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE(
      "InertialOdometry received IMU measurements: " << msg->header.stamp);

  // push imu message onto buffer
  imu_buffer_.push(*msg);

  // return if its not initialized_
  if (!initialized_) return;

  const auto curr_msg = imu_buffer_.front();
  const auto curr_stamp = curr_msg.header.stamp;

  // process each imu message as it comes in
  imu_preint_->AddToBuffer(curr_msg);

  // get relative pose and publish
  ComputeRelativeMotion(prev_stamp_, curr_stamp);

  // get world pose and publish
  ComputeAbsolutePose(curr_stamp);

  odom_seq_++;
  prev_stamp_ = curr_stamp;
  imu_buffer_.pop();
}

void InertialOdometry::processOdometry(
    const nav_msgs::Odometry::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE(
      "InertialOdometry received Odometry measurements: " << msg->header.stamp);

  // return if its not initialized_
  if (!initialized_) return;

  // add constraint at time
  auto transaction =
      imu_preint_->RegisterNewImuPreintegratedFactor(msg->header.stamp);
  sendTransaction(transaction);
}

void InertialOdometry::ComputeRelativeMotion(const ros::Time& prev_stamp,
                                             const ros ::Time& curr_stamp) {
  const auto [T_IMUprev_IMUcurr, cov_rel] =
      imu_preint_->GetRelativeMotion(prev_stamp, curr_stamp);
  T_ODOM_IMUprev_ = T_ODOM_IMUprev_ * T_IMUprev_IMUcurr;
  auto odom_msg_rel = bs_common::TransformToOdometryMessage(
      curr_stamp, odom_seq_, extrinsics_.GetWorldFrameId(),
      extrinsics_.GetImuFrameId(), T_ODOM_IMUprev_, cov_rel);
  relative_odom_publisher_.publish(odom_msg_rel);
}

void InertialOdometry::ComputeAbsolutePose(const ros::Time& curr_stamp) {
  // get world pose and publish
  const auto [T_WORLD_IMU, cov_abs] = imu_preint_->GetPose(curr_stamp);
  auto odom_msg_abs = bs_common::TransformToOdometryMessage(
      curr_stamp, odom_seq_, extrinsics_.GetWorldFrameId(),
      extrinsics_.GetImuFrameId(), T_WORLD_IMU, cov_abs);
  // TODO: what frame do we publish this in? can't have two world frames
  // world_odom_publisher_.publish(odom_msg_abs);
}

void InertialOdometry::onGraphUpdate(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  ROS_INFO_STREAM_ONCE("InertialOdometry recieved initial graph.");
  if (initialized_) {
    imu_preint_->UpdateGraph(graph_msg);
    return;
  }
  std::set<ros::Time> timestamps = bs_common::CurrentTimestamps(graph_msg);

  // publish poses in initial graph as odometry
  for (const auto& stamp : timestamps) {
    T_ODOM_IMUprev_ =
        FusePoseToEigenTransform(*bs_common::GetPosition(graph_msg, stamp),
                                 *bs_common::GetOrientation(graph_msg, stamp));
    auto odom_msg = bs_common::TransformToOdometryMessage(
        stamp, odom_seq_, extrinsics_.GetWorldFrameId(),
        extrinsics_.GetImuFrameId(), T_ODOM_IMUprev_,
        Eigen::Matrix<double, 6, 6>::Identity());
    relative_odom_publisher_.publish(odom_msg);
  }

  // get most recent variables in graph to initialize imu preintegrator
  const ros::Time most_recent_stamp = *timestamps.crbegin();
  auto accel_bias = bs_common::GetAccelBias(graph_msg, most_recent_stamp);
  auto gyro_bias = bs_common::GetGryoscopeBias(graph_msg, most_recent_stamp);
  auto velocity = bs_common::GetVelocity(graph_msg, most_recent_stamp);
  auto position = bs_common::GetPosition(graph_msg, most_recent_stamp);
  auto orientation = bs_common::GetOrientation(graph_msg, most_recent_stamp);

  // create imu preint object
  Eigen::Vector3d ba(accel_bias->x(), accel_bias->y(), accel_bias->z());
  Eigen::Vector3d bg(gyro_bias->x(), gyro_bias->y(), gyro_bias->z());
  imu_preint_ =
      std::make_shared<bs_models::ImuPreintegration>(imu_params_, bg, ba);

  // remove measurements before the start
  while (imu_buffer_.front().header.stamp < most_recent_stamp &&
         !imu_buffer_.empty()) {
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
    ComputeRelativeMotion(prev_stamp_, curr_stamp);

    // get world pose and publish
    ComputeAbsolutePose(curr_stamp);

    odom_seq_++;
    prev_stamp_ = curr_stamp;
    imu_buffer_.pop();
  }

  initialized_ = true;
}

} // namespace bs_models