#pragma once

#include <iostream>
#include <string>

#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>
#include <tf/transform_listener.h>

#include <lidar_aggregation/lidar_aggregators/lidar_aggregator_base.h>

namespace lidar_aggregation {

/**
 * @brief this nodelet subscribes to an odometry topic to retrieve the pose
 * timing, then publishes a ros time at constant intervals (closest to the
 * interval as possible given the odometry timestamps) to signal a lidar_aggregator
 * to start aggregation.
 */
class LidarAggregationTimePublisherNodelet : public nodelet::Nodelet {
public:
  // Nodelet Constructor
  // MUST NEVER FAIL
  LidarAggregationTimePublisherNodelet();

  struct Params {
    std::string odometry_topic;
    std::string aggregation_time_topic;
    ros::Duration aggregation_time;
  };

private:
  void onInit();

  void LoadParams();

  void OdometryCallback(const nav_msgs::OdometryConstPtr message);

  // Nodehandles, both public and private
  ros::NodeHandle nh_, private_nh_;

  // Publisher
  ros::Publisher time_publisher_;

  // Subscibers
  ros::Subscriber odometry_subscriber_;

  // member variables
  Params params_;
  int counter_{0};
  ros::Time prev_time_{ros::Time(0)};
};

} // namespace lidar_aggregation
