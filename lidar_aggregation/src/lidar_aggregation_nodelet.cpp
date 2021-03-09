#include <lidar_aggregation/lidar_aggregation_nodelet.h>

#include <stdlib.h>
#include <string>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>

#include <beam_utils/optional.h>
#include <beam_utils/pointclouds.h>

#include <lidar_aggregation/lidar_aggregators/end_time_lidar_aggregator.h>

namespace lidar_aggregation {

LidarAggregationNodelet::LidarAggregationNodelet() {}

void LidarAggregationNodelet::onInit() {
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  LoadParams();

  aggregate_publisher_ =
      nh_.advertise<sensor_msgs::PointCloud2>(params_.aggregate_topic, 1000);
  aggregation_time_subscriber_ = nh_.subscribe<std_msgs::Time>(
      params_.aggregation_time_topic, 10,
      boost::bind(&LidarAggregationNodelet::AggregationTimeCallback, this, _1));
  pointcloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(
      params_.pointcloud_topic, 10,
      boost::bind(&LidarAggregationNodelet::PointCloudCallback, this, _1));
  odometry_subscriber_ = nh_.subscribe<nav_msgs::Odometry>(
      params_.odometry_topic, 100,
      boost::bind(&LidarAggregationNodelet::OdometryCallback, this, _1));

  // initialize pointers
  poses_ = std::make_shared<tf2::BufferCore>(ros::Duration(poses_buffer_time_));
  extrinsics_ =
      std::make_shared<tf2::BufferCore>(ros::Duration(extrinsics_buffer_time_));

  if (params_.aggregator_type == "ENDTIME") {
    aggregator_ = std::make_unique<EndTimeLidarAggregator>(
        poses_, extrinsics_, first_pose_time_, params_.baselink_frame, params_.lidar_frame,
        world_frame_, ros::Duration(params_.max_aggregation_time_seconds),
        !params_.dynamic_extrinsics, params_.clear_queue_on_update);
  } else if (params_.aggregator_type == "CENTERTIME") {
    throw std::invalid_argument{"CENTERLINE Aggregator not yet implemented."};
  } else {
    ROS_ERROR("Invalid aggregator type: %s, Options: ENDTIME, CENTERTIME",
              params_.aggregator_type.c_str());
    throw std::invalid_argument{"Invalid Aggregator Type"};
  }
}

void LidarAggregationNodelet::LoadParams() {
  if (nh_.getParam("lidar_aggregation/aggregator/aggregator_type",
                   params_.aggregator_type)) {
    ROS_INFO("Loaded parameter aggregator_type: %s",
             params_.aggregator_type.c_str());
  } else {
    ROS_ERROR("Could not load parameter aggregator_type");
    throw std::invalid_argument{"Could not load parameter aggregator_type"};
  }

  if (nh_.getParam("lidar_aggregation/aggregator/aggregation_time_topic",
                   params_.aggregation_time_topic)) {
    ROS_INFO("Loaded parameter aggregation_time_topic: %s",
             params_.aggregation_time_topic.c_str());
  } else {
    ROS_ERROR("Could not load parameter aggregation_time_topic");
    throw std::invalid_argument{
        "Could not load parameter aggregation_time_topic"};
  }

  if (nh_.getParam("lidar_aggregation/aggregator/pointcloud_topic",
                   params_.pointcloud_topic)) {
    ROS_INFO("Loaded parameter pointcloud_topic: %s",
             params_.pointcloud_topic.c_str());
  } else {
    ROS_ERROR("Could not load parameter pointcloud_topic");
    throw std::invalid_argument{"Could not load parameter pointcloud_topic"};
  }

  if (nh_.getParam("lidar_aggregation/aggregator/aggregate_topic",
                   params_.aggregate_topic)) {
    ROS_INFO("Loaded parameter aggregate_topic: %s",
             params_.aggregate_topic.c_str());
  } else {
    ROS_ERROR("Could not load parameter aggregate_topic");
    throw std::invalid_argument{"Could not load parameter aggregate_topic"};
  }

  if (nh_.getParam("lidar_aggregation/aggregator/odometry_topic",
                   params_.odometry_topic)) {
    ROS_INFO("Loaded parameter odometry_topic: %s",
             params_.odometry_topic.c_str());
  } else {
    ROS_ERROR("Could not load parameter odometry_topic");
    throw std::invalid_argument{"Could not load parameter odometry_topic"};
  }

  if (nh_.getParam("lidar_aggregation/aggregator/dynamic_extrinsics",
                   params_.dynamic_extrinsics)) {
    ROS_INFO("Loaded parameter dynamic_extrinsics: %d",
             params_.dynamic_extrinsics);
  } else {
    params_.dynamic_extrinsics = false;
    ROS_INFO("Could not load parameter dynamic_extrinsics, setting to false");
  }

  if (nh_.getParam("lidar_aggregation/aggregator/clear_queue_on_update",
                   params_.dynamic_extrinsics)) {
    ROS_INFO("Loaded parameter clear_queue_on_update: %d",
             params_.clear_queue_on_update);
  } else {
    params_.clear_queue_on_update = false;
    ROS_INFO(
        "Could not load parameter clear_queue_on_update, setting to false");
  }

  if (nh_.getParam("lidar_aggregation/aggregator/max_aggregation_time_seconds",
                   params_.max_aggregation_time_seconds)) {
    ROS_INFO("Loaded parameter max_aggregation_time_seconds: %.2f",
             params_.max_aggregation_time_seconds);
  } else {
    params_.max_aggregation_time_seconds = 10;
    ROS_INFO("Could not load parameter max_aggregation_time_seconds, using 10");
  }

  if (nh_.getParam("lidar_aggregation/aggregator/baselink_frame",
                   params_.baselink_frame)) {
    ROS_INFO("Loaded parameter baselink_frame: %s",
             params_.baselink_frame.c_str());
  } else {
    params_.baselink_frame = "";
    ROS_INFO("Could not load parameter baselink_frame, using frame from "
             "odometry message.");
  }

  if (nh_.getParam("lidar_aggregation/aggregator/lidar_frame",
                   params_.lidar_frame)) {
    ROS_INFO("Loaded parameter lidar_frame: %s", params_.lidar_frame.c_str());
  } else {
    params_.lidar_frame = "";
    ROS_INFO("Could not load parameter lidar_frame, using frame from "
             "input pointcloud message.");
  }

  std::string log_level;
  if (nh_.getParam("lidar_aggregation/aggregator/log_level", log_level)) {
    ROS_INFO("Loaded parameter log_level: %s", log_level.c_str());
  } else {
    log_level = "INFO";
    ROS_INFO("Could not load parameter log_level, using: INFO");
  }
  if (log_level == "INFO") {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Info);
  } else if (log_level == "DEBUG") {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Debug);
  } else if (log_level == "WARN") {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Warn);
  } else if (log_level == "ERROR") {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Error);
  } else if (log_level == "FATAL") {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Fatal);
  } else {
    ROS_ERROR("Invalid log_level, options: DEBUG, INFO, WARN, ERROR, FATAL.");
  }
}

bool LidarAggregationNodelet::SetExtrinsics() {
  // we will not add the extrinsics to the TfTree if they are dynamic, because
  // it'll need to be looked up each time we extract a pose
  if (params_.dynamic_extrinsics) { return true; }

  if (extrinsics_set_) { return true; }

  // if baselink_frame or lidar_frame params not set, we will have to wait
  // till we get the first odometry and pointcloud messages to set the
  // extrinsics
  if (params_.baselink_frame.empty() || params_.lidar_frame.empty()) {
    return false;
  }

  if (AddExtrinsic(ros::Time(0), 5)) {
    ROS_DEBUG("Successfully set static extrinsics");
    extrinsics_set_ = true;
    return true;
  }

  return false;
}

void LidarAggregationNodelet::AggregationTimeCallback(
    const std_msgs::TimeConstPtr message) {
  ROS_DEBUG("Received aggregation time message.");
  if (!SetExtrinsics()) {
    ROS_WARN("Extrinsics not yet set, not creating aggregate. Make sure your "
             "extrinsics are being published to tf.");
    return;
  }

  ROS_DEBUG("Creating aggregate");
  aggregator_->Aggregate(message->data);
  ROS_DEBUG("Getting aggregates");
  std::vector<LidarAggregate> aggregates = aggregator_->Get();

  for (LidarAggregate aggregate : aggregates) {
    if(aggregate.cloud->size() == 0){continue;}
    sensor_msgs::PointCloud2 output_cloud = beam::PCLToROS(
        aggregate.cloud, aggregate.time, params_.lidar_frame, counter_);
    counter_++;
    aggregate_publisher_.publish(output_cloud);
    ROS_DEBUG("Published aggregate no. %d", counter_);
  }
}

void LidarAggregationNodelet::OdometryCallback(
    const nav_msgs::OdometryConstPtr message) {
  ROS_DEBUG("Aggregator received odometry message.");
  if (params_.baselink_frame.empty()) {
    params_.baselink_frame = message->child_frame_id;
    if (params_.baselink_frame.substr(0, 1) == "/") {
      params_.baselink_frame.erase(0, 1);
    }
    ROS_DEBUG("Set base_link frame.");
  }

  if (world_frame_.empty()) {
    world_frame_ = message->header.frame_id;
    if (world_frame_.substr(0, 1) == "/") { world_frame_.erase(0, 1); }
    ROS_DEBUG("Set world frame.");
  }

  if (first_pose_time_ == ros::Time(0)) {
      first_pose_time_ = message->header.stamp;
  }

  geometry_msgs::TransformStamped tf_stamped;
  tf_stamped.header = message->header;
  tf_stamped.header.frame_id = world_frame_;
  tf_stamped.child_frame_id = params_.baselink_frame;
  tf_stamped.transform.translation.x = message->pose.pose.position.x;
  tf_stamped.transform.translation.y = message->pose.pose.position.y;
  tf_stamped.transform.translation.z = message->pose.pose.position.z;
  tf_stamped.transform.rotation = message->pose.pose.orientation;
  std::string authority{"odometry"};
  poses_->setTransform(tf_stamped, authority, false);
  ROS_DEBUG("Added new pose with time: %.10f.", message->header.stamp.toSec());
}

void LidarAggregationNodelet::PointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr message) {
  ROS_DEBUG("Received pointcloud message.");
  if (params_.lidar_frame.empty()) {
    params_.lidar_frame = message->header.frame_id;
        if (params_.lidar_frame.substr(0, 1) == "/") {
      params_.lidar_frame.erase(0, 1);
    }
    ROS_DEBUG("Set lidar_frame frame.");
  }

  if (params_.dynamic_extrinsics) {
    ROS_DEBUG("Adding dynamic extrinsic.");
    AddExtrinsic(message->header.stamp);
  }
  LidarChunk chunk;
  chunk.cloud = beam::ROSToPCL(*message);
  chunk.time = message->header.stamp;
  aggregator_->Add(chunk);
  ROS_DEBUG("Added lidar chunk.");
}

// TODO: use beam_common::PoseLookup class to cleanup this code
bool LidarAggregationNodelet::AddExtrinsic(const ros::Time& time,
                                           int num_attempts) {
  // if baselink and lidar frames are the same, no extrinsics needed                                             
  if(params_.baselink_frame == params_.lidar_frame){return true;}   

  tf::StampedTransform T_BASELINK_LIDAR;
  int lookup_counter = 0;
  while (lookup_counter < num_attempts) {
    try {
      tf_listener_.lookupTransform(params_.baselink_frame, params_.lidar_frame,
                                   time, T_BASELINK_LIDAR);
      break;
    } catch (tf::TransformException& ex) {
      lookup_counter++;
      if (lookup_counter == num_attempts) {
        ROS_WARN("Cannot lookup extrinsics for t = %.10f", time.toSec());
        return false;
      }
      ROS_WARN("Cannot lookup extrinsics for t = %.10f, trying again in 1s",
               time.toSec());
      ros::Duration(1.0).sleep();
    }
  }

  std::string autority{"tf"};
  geometry_msgs::TransformStamped T;
  T.transform.translation.x = T_BASELINK_LIDAR.getOrigin().getX();
  T.transform.translation.y = T_BASELINK_LIDAR.getOrigin().getY();
  T.transform.translation.z = T_BASELINK_LIDAR.getOrigin().getZ();
  T.transform.rotation.x = T_BASELINK_LIDAR.getRotation().getX();
  T.transform.rotation.y = T_BASELINK_LIDAR.getRotation().getY();
  T.transform.rotation.z = T_BASELINK_LIDAR.getRotation().getZ();
  T.transform.rotation.w = T_BASELINK_LIDAR.getRotation().getW();
  T.child_frame_id = T_BASELINK_LIDAR.child_frame_id_;
  T.header.frame_id = T_BASELINK_LIDAR.frame_id_;
  T.header.stamp = T_BASELINK_LIDAR.stamp_;
  extrinsics_->setTransform(T, autority, !params_.dynamic_extrinsics);
  ROS_DEBUG("Added extrinsic.");

  return true;
}

} // namespace lidar_aggregation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lidar_aggregation::LidarAggregationNodelet,
                       nodelet::Nodelet);
