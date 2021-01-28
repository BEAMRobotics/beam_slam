#include <lidar_aggregation/lidar_aggregation_nodelet.hpp>

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

#include <beam_utils/optional.h>
#include <beam_utils/pointclouds.h>

#include <lidar_aggregation/lidar_aggregators/end_time_lidar_aggregator.hpp>

namespace lidar_aggregation {

LidarAggregationNodelet::LidarAggregationNodelet() {}

void LidarAggregationNodelet::onInit() {
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  loadParams();

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

  // Set nodelet's internal logger
  SetInternalLogger();

  // add extrinsics to TfTree
  SetExtrinsics();

  // initialize pointers
  poses_ = std::make_shared<tf2::BufferCore>(ros::Duration(poses_buffer_time_));
  extrinsics_ =
      std::make_shared<tf2::BufferCore>(ros::Duration(extrinsics_buffer_time_));

  if (params_.aggregator_type == "ENDTIME") {
    // TODO: figure this out for the case where we don't know
    aggregator_ = std::make_unique<EndTimeLidarAggregator>(poses_, extrinsics_, "test", "test", "test");
  } else if (params_.aggregator_type == "CENTERTIME") {
    throw std::invalid_argument{"CENTERLINE Aggregator not yet implemented."};
  } else {
    ROS_ERROR("Invalid aggregator type: %s, Options: ENDTIME, CENTERTIME",
              params_.aggregator_type);
    throw std::invalid_argument{"Invalid Aggregator Type"};
  }
  
}

void LidarAggregationNodelet::loadParams() {
  if (nh_.getParam("lidar_aggregation/aggregation_time_topic",
                   params_.aggregation_time_topic)) {
    ROS_INFO("Loaded parameter aggregation_time_topic: %s",
             params_.aggregation_time_topic);
  } else {
    ROS_ERROR("Could not load parameter aggregation_time_topic");
  }

  if (nh_.getParam("lidar_aggregation/pointcloud_topic",
                   params_.pointcloud_topic)) {
    ROS_INFO("Loaded parameter pointcloud_topic: %s", params_.pointcloud_topic);
  } else {
    ROS_ERROR("Could not load parameter pointcloud_topic");
  }

  if (nh_.getParam("lidar_aggregation/aggregate_topic",
                   params_.aggregate_topic)) {
    ROS_INFO("Loaded parameter aggregate_topic: %s", params_.aggregate_topic);
  } else {
    ROS_ERROR("Could not load parameter aggregate_topic");
  }

  if (nh_.getParam("lidar_aggregation/odometry_topic",
                   params_.odometry_topic)) {
    ROS_INFO("Loaded parameter odometry_topic: %s", params_.odometry_topic);
  } else {
    ROS_ERROR("Could not load parameter odometry_topic");
  }

  if (nh_.getParam("lidar_aggregation/log_directory", params_.log_directory)) {
    ROS_INFO("Loaded parameter log_directory: %s", params_.log_directory);
  } else {
    params_.log_directory = ".ros/log/";
    ROS_INFO("Could not parameter load log_directory, using .ros/log");
  }

  if (nh_.getParam("lidar_aggregation/dynamic_extrinsics",
                   params_.dynamic_extrinsics)) {
    ROS_INFO("Loaded parameter dynamic_extrinsics: %d",
             params_.dynamic_extrinsics);
  } else {
    params_.dynamic_extrinsics = false;
    ROS_INFO("Could not load parameter dynamic_extrinsics, setting to false");
  }

  if (nh_.getParam("lidar_aggregation/clear_queue_on_update",
                   params_.dynamic_extrinsics)) {
    ROS_INFO("Loaded parameter clear_queue_on_update: %d",
             params_.clear_queue_on_update);
  } else {
    params_.clear_queue_on_update = false;
    ROS_INFO(
        "Could not load parameter clear_queue_on_update, setting to false");
  }

  if (nh_.getParam("lidar_aggregation/max_aggregation_time_seconds",
                   params_.max_aggregation_time_seconds)) {
    ROS_INFO("Loaded parameter clear_queue_on_update: %d",
             params_.max_aggregation_time_seconds);
  } else {
    params_.max_aggregation_time_seconds = 10;
    ROS_INFO("Could not load parameter max_aggregation_time_seconds, using 10");
  }

  if (nh_.getParam("lidar_aggregation/baselink_frame",
                   params_.baselink_frame)) {
    ROS_INFO("Loaded parameter baselink_frame: %s", params_.baselink_frame);
  } else {
    params_.baselink_frame = "";
    ROS_INFO("Could not load parameter baselink_frame, using frame from "
             "odometry message.");
  }

  if (nh_.getParam("lidar_aggregation/lidar_frame", params_.lidar_frame)) {
    ROS_INFO("Loaded parameter lidar_frame: %s", params_.lidar_frame);
  } else {
    params_.lidar_frame = "";
    ROS_INFO("Could not load parameter lidar_frame, using frame from "
             "input pointcloud message.");
  }
}

// Set a logger to catch the errors published by the internal C++ library so
// that you don't spam the console!
void LidarAggregationNodelet::SetInternalLogger() {
  boost::log::add_common_attributes();

  // Set the logging directory
  // TODO(msmart) clean this up so that it doesn't use '/' convention for
  // files going to HOME. Maybe check for first character in parameter being
  // '/' and then treat as absolute, or relative if '/' is not first
  // character.
  std::string log_path = getenv("HOME") + params_.log_directory;

  // TODO(msmart) add more complicated example demonstrating tiers of severity
  // logging. For now - just drop anything less than info.

  // Filter based on logging severity
  boost::log::core::get()->set_filter(
      // trace and debug level log events are filtered out
      boost::log::trivial::severity >= boost::log::trivial::info);

  // Define the desired log file.
  // TODO(msmart) - fix this formatting so lint stops yelling.
  boost::log::add_file_log(
      // Set log file name
      boost::log::keywords::file_name =
          log_path + "/lidar_aggregation_nodelet_%N.log",
      // Set log file rotation size in bytes.
      // These values are low here for example.
      boost::log::keywords::rotation_size = 512,
      // Set log entry format
      boost::log::keywords::format = "[%TimeStamp%]: %Message%"
      // Call make_collector off of the sink created by add_file_log
      )
      ->locked_backend()
      ->set_file_collector(
          // Collectors are only applied after a log file is closed.
          // Each time a log file is closed, the collector checks to
          // see if an action needs to be taken relative to the next
          // log file.
          boost::log::sinks::file::make_collector(
              // 'target' sets the folder that will be "managed"
              // by overwriting logs to maintain following
              // objective.
              boost::log::keywords::target = log_path,
              // If the logs being created in total exceed
              // max_size, then the next log file created will
              // overwrite the first log file.
              boost::log::keywords::max_size = 5 * 512));
}

void LidarAggregationNodelet::SetExtrinsics() {
  // we will not add the extrinsics to the TfTree if they are dynamic, because
  // it'll need to be looked up each time we extract a pose
  if (params_.dynamic_extrinsics) { return; }

  // if baselink_frame or lidar_frame params not set, we will have to wait
  // till we get the first odometry and pointcloud messages to set the
  // extrinsics
  if (params_.baselink_frame.empty() || params_.lidar_frame.empty()) { return; }

  AddExtrinsic(ros::Time(0));
}

void LidarAggregationNodelet::AggregationTimeCallback(
    const std_msgs::TimeConstPtr message) {
  aggregator_->Aggregate(message->data);
  std::vector<LidarAggregate> aggregates = aggregator_->Get();

  for (LidarAggregate aggregate : aggregates) {
    sensor_msgs::PointCloud2 output_cloud = beam::PCLToROS(
        aggregate.cloud, aggregate.time, params_.lidar_frame, counter_);
    counter_++;
    aggregate_publisher_.publish(output_cloud);
  }
}

void LidarAggregationNodelet::OdometryCallback(
    const nav_msgs::OdometryConstPtr message) {
  if (params_.baselink_frame.empty()) {
    params_.baselink_frame = message->child_frame_id;
  }

  if (world_frame_.empty()) { world_frame_ = message->header.frame_id; }

  geometry_msgs::TransformStamped tf_stamped;
  tf_stamped.header = message->header;
  tf_stamped.child_frame_id = message->child_frame_id;
  tf_stamped.transform.translation.x = message->pose.pose.position.x;
  tf_stamped.transform.translation.y = message->pose.pose.position.y;
  tf_stamped.transform.translation.z = message->pose.pose.position.z;
  tf_stamped.transform.rotation = message->pose.pose.orientation;
  std::string authority{"odometry"};
  poses_->setTransform(tf_stamped, authority);
}

void LidarAggregationNodelet::PointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr message) {
  if (params_.lidar_frame.empty()) {
    params_.lidar_frame = message->header.frame_id;
    AddExtrinsic(ros::Time(0));
  }

  if (params_.dynamic_extrinsics) { AddExtrinsic(message->header.stamp); }

  LidarChunk chunk;
  chunk.cloud = beam::ROSToPCL(*message);
  chunk.time = message->header.stamp;
  aggregator_->Add(chunk);
}

void LidarAggregationNodelet::AddExtrinsic(const ros::Time& time) {
  tf::StampedTransform T_BASELINK_LIDAR;
  try {
    tf_listener_.lookupTransform(params_.baselink_frame, params_.lidar_frame,
                                 time, T_BASELINK_LIDAR);
  } catch (tf::TransformException& ex) {
    ROS_WARN("Cannot lookup extrinsics for t = .10f", time.toSec());
  }
  std::string autority{"tf"};
  geometry_msgs::TransformStamped T;
  // T.transform.translation.x = 
  // T.transform.translation.y = 
  // T.transform.translation.z = 
  // T.transform.quaternion = 
  // T.setData(tf::Transform(T_BASELINK_LIDAR.transform.rotation, T_BASELINK_LIDAR.transform.translation));
  T.child_frame_id = T_BASELINK_LIDAR.child_frame_id_;
  T.header.frame_id = T_BASELINK_LIDAR.frame_id_;
  T.header.stamp = T_BASELINK_LIDAR.stamp_;
  extrinsics_->setTransform(T, autority, !params_.dynamic_extrinsics);
}

} // namespace lidar_aggregation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lidar_aggregation::LidarAggregationNodelet,
                       nodelet::Nodelet);
