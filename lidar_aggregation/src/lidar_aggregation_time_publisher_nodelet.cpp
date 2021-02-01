#include <lidar_aggregation/lidar_aggregation_time_publisher_nodelet.h>

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

#include <lidar_aggregation/lidar_aggregators/end_time_lidar_aggregator.h>

namespace lidar_aggregation {

LidarAggregationTimePublisherNodelet::LidarAggregationTimePublisherNodelet() {}

void LidarAggregationTimePublisherNodelet::onInit() {
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();
  
  LoadParams();
  
  time_publisher_ =
      nh_.advertise<std_msgs::Time>(params_.aggregation_time_topic, 1000);
  odometry_subscriber_ = nh_.subscribe<nav_msgs::Odometry>(
      params_.odometry_topic, 100,
      boost::bind(&LidarAggregationTimePublisherNodelet::OdometryCallback, this,
                  _1));
}

void LidarAggregationTimePublisherNodelet::LoadParams() {
  if (nh_.getParam("lidar_aggregation/aggregation_time_topic",
                   params_.aggregation_time_topic)) {
    ROS_INFO("Loaded parameter aggregation_time_topic: %s",
             params_.aggregation_time_topic.c_str());
  } else {
    ROS_ERROR("Could not load parameter aggregation_time_topic, using default: "
              "default_topic");
    throw std::invalid_argument{
        "Could not load parameter aggregation_time_topic"};
  }

  if (nh_.getParam("lidar_aggregation/odometry_topic",
                   params_.odometry_topic)) {
    ROS_INFO("Loaded parameter odometry_topic: %s", params_.odometry_topic.c_str());
  } else {
    ROS_ERROR("Could not load parameter odometry_topic");
    throw std::invalid_argument{"Could not load parameter odometry_topic"};
  }

  double aggregation_time_seconds;
  if (nh_.getParam("lidar_aggregation/aggregation_time_seconds",
                   aggregation_time_seconds)) {
    ROS_INFO("Loaded parameter aggregation_time: %.2f", aggregation_time_seconds);
    params_.aggregation_time = ros::Duration(aggregation_time_seconds);
  } else {
    params_.aggregation_time = ros::Duration(1);
    ROS_INFO("Could not load parameter aggregation_time_seconds, using: 1");
  }
}

void LidarAggregationTimePublisherNodelet::OdometryCallback(
    const nav_msgs::OdometryConstPtr message) {
  if(message->header.stamp - prev_time_ > params_.aggregation_time){
    std_msgs::Time time;
    time.data = message->header.stamp;
    time_publisher_.publish(time);
    prev_time_ = message->header.stamp;
  }
  
}

} // namespace lidar_aggregation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lidar_aggregation::LidarAggregationTimePublisherNodelet,
                       nodelet::Nodelet);
