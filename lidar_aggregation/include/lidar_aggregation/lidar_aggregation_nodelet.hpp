#pragma once

#include <iostream>
#include <string>

#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>
#include <tf/transform_listener.h>

#include <lidar_aggregation/lidar_aggregators/lidar_aggregator_base.hpp>

namespace lidar_aggregation {

class LidarAggregationNodelet : public nodelet::Nodelet {
public:
  // Nodelet Constructor
  // MUST NEVER FAIL
  LidarAggregationNodelet();

  struct Params {
    std::string aggregator_type; // OPTIONS: ENDTIME, CENTERTIME
    std::string aggregation_time_topic;
    std::string pointcloud_topic; // input cloud topic
    std::string aggregate_topic;
    std::string odometry_topic;
    std::string log_directory; // relative to the $HOME directory
    bool dynamic_extrinsics;
    bool clear_queue_on_update;
    double max_aggregation_time_seconds;
    std::string baselink_frame; // if not set, it will use frame from odometry
    std::string lidar_frame;    // input and aggregate cloud frame. If not
                                // set, it will use frame from pointcloud_topic
  };

private:
  void onInit();

  void loadParams();

  // Set logging sink to be used by boost::log.
  //      Only used for code internal to the nodelet (non-ROS code)
  void SetInternalLogger();

  void SetExtrinsics();

  // lookup extrinsic calibration on /tf and save to extrinsics
  void AddExtrinsic(const ros::Time& time);

  // callbacks
  void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr message);
  void OdometryCallback(const nav_msgs::OdometryConstPtr message);
  void AggregationTimeCallback(const std_msgs::TimeConstPtr message);

  // Nodehandles, both public and private
  ros::NodeHandle nh_, private_nh_;

  // Publisher
  ros::Publisher aggregate_publisher_;

  // Subscibers
  ros::Subscriber aggregation_time_subscriber_;
  ros::Subscriber pointcloud_subscriber_;
  ros::Subscriber odometry_subscriber_;
  tf::TransformListener tf_listener_;

  // member variables
  Params params_;
  std::unique_ptr<LidarAggregatorBase> aggregator_;
  std::shared_ptr<tf2::BufferCore> poses_;
  std::shared_ptr<tf2::BufferCore> extrinsics_;
  Eigen::Matrix4d T_BASELINK_LIDAR_;
  int counter_{0};
  std::string world_frame_;

  // other tunable parameters
  // int max_lookup_attempts_{3};
  double poses_buffer_time_{100};
  double extrinsics_buffer_time_{100};
};

} // namespace lidar_aggregation
