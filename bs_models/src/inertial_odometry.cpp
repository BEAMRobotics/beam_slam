#include <bs_models/inertial_odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>

#include <bs_common/conversions.h>
#include <bs_common/graph_access.h>
#include <bs_constraints/inertial/relative_imu_state_3d_stamped_constraint.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::InertialOdometry, fuse_core::SensorModel);

namespace bs_models {

InertialOdometry::InertialOdometry()
    : fuse_core::AsyncSensorModel(2),
      throttled_imu_callback_(std::bind(&InertialOdometry::processIMU, this,
                                        std::placeholders::_1)),
      throttled_trigger_callback_(std::bind(&InertialOdometry::processTrigger,
                                            this, std::placeholders::_1)) {}

void InertialOdometry::onInit() {
  // Read settings from the parameter sever
  calibration_params_.loadFromROS();
  params_.loadFromROS(private_node_handle_);

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

  trigger_subscriber_ = private_node_handle_.subscribe<std_msgs::Time>(
      ros::names::resolve("/local_mapper/inertial_odometry/trigger"), 10,
      &ThrottledTriggerCallback::callback, &throttled_trigger_callback_,
      ros::TransportHints().tcpNoDelay(false));

  // setup publishers
  odometry_publisher_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odometry", 100);
  pose_publisher_ =
      private_node_handle_.advertise<geometry_msgs::PoseStamped>("pose", 100);
}

void InertialOdometry::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE(
      "InertialOdometry received IMU measurements: " << msg->header.stamp);

  // return if its not initialized_
  if (!initialized_) {
    // push imu message onto buffer
    imu_buffer_.push(msg);
    ROS_INFO_THROTTLE(
        1, "inertial odometry not yet initialized, waiting on first graph "
           "update before beginning");
    return;
  }

  // process each imu message as it comes in
  imu_preint_->AddToBuffer(*msg);

  // get relative pose and publish
  ComputeRelativeMotion(prev_stamp_, msg->header.stamp);

  // get world pose and publish
  ComputeAbsolutePose(msg->header.stamp);

  odom_seq_++;
  prev_stamp_ = msg->header.stamp;
}

void InertialOdometry::processTrigger(const std_msgs::Time::ConstPtr& msg) {
  // return if its not initialized_
  if (!initialized_) { return; }

  // add constraint at time
  auto transaction = imu_preint_->RegisterNewImuPreintegratedFactor(msg->data);

  if (!transaction) { return; }

  sendTransaction(transaction);
}

void InertialOdometry::ComputeRelativeMotion(const ros::Time& prev_stamp,
                                             const ros ::Time& curr_stamp) {
  Eigen::Vector3d velocity_curr;
  const auto [T_IMUprev_IMUcurr, cov_rel] =
      imu_preint_->GetRelativeMotion(prev_stamp, curr_stamp, velocity_curr);

  // reorder covariance from: roll,pitch,yaw,x,y,z to x,y,z,roll,pitch,yaw
  Eigen::Matrix<double, 6, 6> cov_reordered =
      Eigen::Matrix<double, 6, 6>::Zero();
  cov_reordered.block<3, 3>(0, 0) = cov_rel.block<3, 3>(3, 3);
  cov_reordered.block<3, 3>(0, 3) = cov_rel.block<3, 3>(3, 0);
  cov_reordered.block<3, 3>(3, 0) = cov_rel.block<3, 3>(0, 3);
  cov_reordered.block<3, 3>(3, 3) = cov_rel.block<3, 3>(0, 0);

  // publish relative odometry
  T_ODOM_IMUprev_ = T_ODOM_IMUprev_ * T_IMUprev_IMUcurr;
  auto odom_msg_rel = bs_common::TransformToOdometryMessage(
      curr_stamp, odom_seq_, extrinsics_.GetWorldFrameId(),
      extrinsics_.GetImuFrameId(), T_ODOM_IMUprev_, cov_reordered);
  odometry_publisher_.publish(odom_msg_rel);
}

void InertialOdometry::ComputeAbsolutePose(const ros::Time& curr_stamp) {
  // get world pose and publish
  const auto [T_WORLD_IMU, cov_abs] = imu_preint_->GetPose(curr_stamp);
  geometry_msgs::PoseStamped msg;
  bs_common::EigenTransformToPoseStamped(T_WORLD_IMU, curr_stamp, odom_seq_,
                                         extrinsics_.GetImuFrameId(), msg);
  pose_publisher_.publish(msg);
}

void InertialOdometry::onGraphUpdate(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  if (initialized_) {
    imu_preint_->UpdateGraph(graph_msg);
    return;
  }

  ROS_INFO("InertialOdometry received initial graph.");
  std::set<ros::Time> timestamps = bs_common::CurrentTimestamps(graph_msg);

  // publish poses in initial graph as odometry
  for (const auto& stamp : timestamps) {
    T_ODOM_IMUprev_ =
        FusePoseToEigenTransform(*bs_common::GetPosition(graph_msg, stamp),
                                 *bs_common::GetOrientation(graph_msg, stamp));
    auto odom_msg = bs_common::TransformToOdometryMessage(
        stamp, odom_seq_, extrinsics_.GetWorldFrameId(),
        extrinsics_.GetImuFrameId(), T_ODOM_IMUprev_,
        Eigen::Matrix<double, 6, 6>::Identity() * 1e-5);
    odometry_publisher_.publish(odom_msg);
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
  imu_preint_ = std::make_shared<bs_models::ImuPreintegration>(
      imu_params_, bg, ba, params_.inertial_info_weight, false);

  // remove measurements before the start
  while (imu_buffer_.front()->header.stamp < most_recent_stamp &&
         !imu_buffer_.empty()) {
    imu_buffer_.pop();
  }

  // set the start
  imu_preint_->SetStart(most_recent_stamp, orientation, position, velocity);

  // iterate through existing buffer, estimate poses and output
  prev_stamp_ = most_recent_stamp;
  while (!imu_buffer_.empty()) {
    const auto& imu_msg = imu_buffer_.front();
    imu_preint_->AddToBuffer(*imu_msg);
    const auto& curr_stamp = imu_msg->header.stamp;

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