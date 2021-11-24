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
 * @brief this nodelet performs lidar_aggregation given any aggregator class
 * derived from lidar_aggregator_base class. To start aggregation, it subscribes
 * to a time topic to signal aggregation times. It also subscribes to tf to
 * retrieve dynamic or static extrinsic transformations published to tf, and
 * retrieves poses from a specified odometry topic to interpolate points.
 */
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
    bool dynamic_extrinsics;
    bool clear_queue_on_update;
    double max_aggregation_time_seconds;
    std::string baselink_frame; // if not set, it will use frame from odometry
    std::string lidar_frame;    // input and aggregate cloud frame. If not
                                // set, it will use frame from pointcloud_topic
  };

private:
  void onInit();

  void LoadParams();

  bool SetExtrinsics();

  // lookup extrinsic calibration on /tf and save to extrinsics
  bool AddExtrinsic(const ros::Time& time, int num_attempts = 1);

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
  bool extrinsics_set_{false};
  ros::Time first_pose_time_{ros::Time(0)};

  // other tunable parameters
  double poses_buffer_time_{100};

  // robot extrinsics
  bs_common::ExtrinsicsLookupOnline &extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
};

} // namespace lidar_aggregation
