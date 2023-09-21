#include <bs_models/gravity_alignment.h>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>

#include <beam_utils/pointclouds.h>

#include <bs_common/conversions.h>
#include <bs_constraints/global/gravity_alignment_stamped_constraint.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::GravityAlignment, fuse_core::SensorModel);

namespace bs_models {

GravityAlignment::GravityAlignment()
    : fuse_core::AsyncSensorModel(2),
      throttled_imu_callback_(std::bind(&GravityAlignment::processIMU, this,
                                        std::placeholders::_1)),
      throttled_odom_callback_(std::bind(&GravityAlignment::processOdometry,
                                         this, std::placeholders::_1)) {}

void GravityAlignment::onInit() {
  // Read settings from the parameter sever
  params_.loadFromROS(private_node_handle_);
  covariance_ = 1 /
                (params_.gravity_information_weight *
                 params_.gravity_information_weight) *
                Eigen::Matrix2d::Identity();
  // check for non empty odom topic
  if (params_.constraint_odom_topic.empty()) {
    ROS_ERROR("Constraint odom topic cannot be empty, must be a valid odometry "
              "topic.");
    throw std::invalid_argument{"Constraint odom topic cannot be empty, must "
                                "be a valid odometry topic."};
  }
}

void GravityAlignment::onStart() {
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

void GravityAlignment::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_buffer_.emplace(msg->header.stamp.toNSec(), msg);
  while (imu_buffer_.rbegin()->first - imu_buffer_.begin()->first >
         buffer_duration_in_ns_) {
    imu_buffer_.erase(imu_buffer_.begin());
  }
}

void GravityAlignment::processOdometry(
    const nav_msgs::Odometry::ConstPtr& msg) {
  int64_t odom_time_ns = msg->header.stamp.toNSec();
  auto lower_bound = imu_buffer_.lower_bound(odom_time_ns);

  // if we get to the end, then IMU data doesn't exist so exit. This shouldn't
  // happen as we always expect the odom trigger to arrive after IMU data. If
  // this assumption isn't true, then something is wrong
  if (lower_bound == imu_buffer_.end()) {
    BEAM_ERROR("No IMU data available for incoming odom timestamp of {}s. Not "
               "adding gravity alignment constraint. Make sure your IMU data "
               "is arriving before your Odometry message",
               msg->header.stamp.toSec());
    return;
  }

  // get closest IMU time
  int64_t closest_imu_time_ns;
  if (lower_bound->first == odom_time_ns) {
    // if exact time is returned, no need to check for closest
    closest_imu_time_ns = lower_bound->first;
  } else if (lower_bound == imu_buffer_.begin()) {
    // if the first time is returned, then check the max time offset
    if (lower_bound->first - odom_time_ns > max_time_offset_in_ns_) {
      // this also shouldn't occur unless the buffer isn't long enough
      BEAM_ERROR(
          "No IMU data available for incoming odom timestamp of {}s. Not "
          "adding gravity alignment constraint. Check the IMU buffer time.",
          msg->header.stamp.toSec());
      return;
    }
    closest_imu_time_ns = lower_bound->first;
  } else {
    // else then we are not at the bounds
    auto prev = std::prev(lower_bound);
    closest_imu_time_ns =
        (odom_time_ns - prev->first > lower_bound->first - odom_time_ns)
            ? lower_bound->first
            : prev->first;
    // double check that there isn't too much time offset (this also shouldn't
    // occur unless we are having major IMU drops)
    if (std::abs(closest_imu_time_ns - odom_time_ns) > max_time_offset_in_ns_) {
      BEAM_ERROR(
          "No IMU data available for incoming odom timestamp of {}s. Not "
          "adding gravity alignment constraint. Check for IMU data drops",
          msg->header.stamp.toSec());
      return;
    }
  }

  AddConstraint(imu_buffer_.at(closest_imu_time_ns), msg->header.stamp);

  // clear all IMU messages before constraint time
  while (!imu_buffer_.empty() &&
         imu_buffer_.begin()->first < closest_imu_time_ns) {
    imu_buffer_.erase(imu_buffer_.begin());
  }

  Publish(imu_buffer_.at(closest_imu_time_ns), msg);
}

void GravityAlignment::AddConstraint(const sensor_msgs::Imu::ConstPtr& imu_msg,
                                     const ros::Time& stamp) {
  auto orientation_uuid = fuse_core::uuid::generate(
      "fuse_variables::Orientation3DStamped", stamp, fuse_core::uuid::NIL);
  Eigen::Quaterniond q_World_Imu(imu_msg->orientation.w, imu_msg->orientation.x,
                                 imu_msg->orientation.y,
                                 imu_msg->orientation.z);
  Eigen::Matrix3d R_Imu_World = q_World_Imu.inverse().toRotationMatrix();
  Eigen::Matrix4d T_Baselink_Imu;
  extrinsics_.GetT_BASELINK_IMU(T_Baselink_Imu);
  Eigen::Matrix3d R_Baselink_World =
      T_Baselink_Imu.block(0, 0, 3, 3) * R_Imu_World;
  Eigen::Vector3d g_in_Baselink = R_Baselink_World * g_in_World_;
  fuse_variables::Orientation3DStamped o_World_Imu(stamp);
  auto constraint =
      bs_constraints::GravityAlignmentStampedConstraint::make_shared(
          source_, orientation_uuid, g_in_Baselink, covariance_);
  auto transaction = std::make_shared<fuse_core::Transaction>();
  transaction->addConstraint(constraint);
  sendTransaction(transaction);
}

void GravityAlignment::Publish(
    const sensor_msgs::Imu::ConstPtr& imu_data,
    const nav_msgs::Odometry::ConstPtr& odom_data) const {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  auto frame = beam::CreateFrameCol(odom_data->header.stamp);

  Eigen::Matrix4d T_World_Baselink;
  bs_common::OdometryMsgToTransformationMatrix(*odom_data, T_World_Baselink);
  beam::MergeFrameToCloud(cloud, frame, T_World_Baselink);

  Eigen::Quaterniond q(imu_data->orientation.w, imu_data->orientation.x,
                       imu_data->orientation.y, imu_data->orientation.z);
  Eigen::Matrix3d R_World_Imu = q.toRotationMatrix();
  Eigen::Vector3d t_World_Baselink = T_World_Baselink.block(0, 3, 3, 1);
  Eigen::Vector3d g_in_Imu = gravity_vector_length_ * params_.gravity_nominal;
  Eigen::Vector3d g_in_world = R_World_Imu * g_in_Imu + t_World_Baselink;
  pcl::PointXYZRGBL p1;
  p1.r = 255;
  p1.g = 0;
  p1.b = 255;
  p1.label = odom_data->header.stamp.toNSec();
  auto p2 = p1;
  p1.x = t_World_Baselink[0];
  p1.y = t_World_Baselink[1];
  p1.z = t_World_Baselink[2];
  p2.x = g_in_world[0];
  p2.y = g_in_world[1];
  p2.z = g_in_world[2];
  auto gravity_cloud = beam::DrawLine<pcl::PointXYZRGBL>(p1, p2);
  cloud += gravity_cloud;

  sensor_msgs::PointCloud2 cloud_msg = beam::PCLToROS<pcl::PointXYZRGBL>(
      cloud, odom_data->header.stamp, extrinsics_.GetWorldFrameId(), counter_);
  publisher_.publish(cloud_msg);
}

} // namespace bs_models