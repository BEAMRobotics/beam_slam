#include <bs_models/gravity_alignmen.h>

#include <pluginlib/class_list_macros.h>

#include <bs_constraints/global/gravity_alignment_constraint.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::InertialOdometry, fuse_core::SensorModel);

namespace bs_models {

InertialOdometry::InertialOdometry()
    : fuse_core::AsyncSensorModel(2),
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
  covariance_ = 1 /
                (params_.gravity_info_weight * params_.gravity_info_weight) *
                Eigen::Matrix2d::Identity();
  // check for non empty odom topic
  if (params_.constraint_odom_topic.empty()) {
    ROS_ERROR("Constraint odom topic cannot be empty, must be a valid odometry "
              "topic.");
    throw std::invalid_argument{"Constraint odom topic cannot be empty, must "
                                "be a valid odometry topic."};
  }
}

void InertialOdometry::onStart() {
  imu_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Imu>(
      ros::names::resolve(params_.imu_topic), 300,
      &ThrottledIMUCallback::callback, &throttled_imu_callback_,
      ros::TransportHints().tcpNoDelay(false));

  odom_subscriber_ = private_node_handle_.subscribe<nav_msgs::Odometry>(
      ros::names::resolve(params_.constraint_odom_topic), 10,
      &ThrottledOdomCallback::callback, &throttled_odom_callback_,
      ros::TransportHints().tcpNoDelay(false));

  // setup publishers
  publisher_ = private_node_handle_.advertise<sensor_msgs::PointCloud2>(
      "alignment_constraint", 10);
}

void InertialOdometry::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_buffer_.push(msg);
  while (imu_buffer_.back()->header.stamp - imu_buffer_.front()->header.stamp >
         queue_duration_) {
    imu_buffer_.pop();
  }
}

void InertialOdometry::processOdometry(
    const nav_msgs::Odometry::ConstPtr& msg) {
  AddConstraint(msg);
}

void InertialOdometry::AddConstraint(
    const nav_msgs::Odometry::ConstPtr& msg) const {
  fuse_variables::Orientation3DStamped o_World_Imu;
  
  // TODO

  GravityAlignmentConstraint constraint(o_World_Imu, covariance_);
}

} // namespace bs_models