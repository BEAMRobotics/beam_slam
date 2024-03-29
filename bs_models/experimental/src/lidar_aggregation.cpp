#include "lidar_aggregation.h"

#include <pluginlib/class_list_macros.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::experimental::LidarAggregation,
                       fuse_core::SensorModel)

namespace bs_models::experimental {

LidarAggregation::LidarAggregation()
    : fuse_core::AsyncSensorModel(1),
      throttled_callback_pc_(std::bind(&LidarAggregation::ProcessPointcloud,
                                       this, std::placeholders::_1)),
      throttled_callback_time_(std::bind(&LidarAggregation::ProcessTimeTrigger,
                                         this, std::placeholders::_1)) {}

void LidarAggregation::onInit() {
  ROS_DEBUG("Initialzing");
  params_.loadFromROS(private_node_handle_);
  ROS_DEBUG("Loaded params");
  max_aggregation_duration_.fromSec(params_.max_aggregation_time_seconds);
  if (params_.lidar_type == LidarType::VELODYNE) {
    velodyne_lidar_aggregator_ = std::make_unique<LidarAggregator<PointXYZIRT>>(
        params_.frame_initializer_config, max_aggregation_duration_);
  } else if (params_.lidar_type == LidarType::OUSTER) {
    ouster_lidar_aggregator_ =
        std::make_unique<LidarAggregator<PointXYZITRRNR>>(
            params_.frame_initializer_config, max_aggregation_duration_);
  } else {
    BEAM_ERROR(
        "Invalid lidar type param. Lidar type may not be implemented yet.");
    throw std::invalid_argument{"Invalid lidar type."};
  }
  ROS_DEBUG("Done initialization");
}

void LidarAggregation::onStart() {
  pointcloud_subscriber_ =
      private_node_handle_.subscribe<sensor_msgs::PointCloud2>(
          ros::names::resolve(params_.pointcloud_topic),
          lidar_subscriber_queue_size_, &ThrottledCallbackPC::callback,
          &throttled_callback_pc_, ros::TransportHints().tcpNoDelay(false));

  aggregation_time_subscriber_ = private_node_handle_.subscribe<std_msgs::Time>(
      ros::names::resolve(params_.aggregation_time_topic),
      aggregation_time_subscriber_queue_size_, &ThrottledCallbackTime::callback,
      &throttled_callback_time_, ros::TransportHints().tcpNoDelay(false));

  aggregate_publisher_ =
      private_node_handle_.advertise<sensor_msgs::PointCloud2>(
          "points_undistorted", aggregate_publisher_queue_size_);
}

void LidarAggregation::onStop() {
  aggregation_time_subscriber_.shutdown();
  pointcloud_subscriber_.shutdown();
}

void LidarAggregation::ProcessPointcloud(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (params_.lidar_type == LidarType::VELODYNE) {
    pcl::PointCloud<PointXYZIRT> cloud;
    beam::ROSToPCL(cloud, *msg);
    std::map<uint64_t, pcl::PointCloud<PointXYZIRT>> sorted_clouds =
        SeparateTimeStamps<PointXYZIRT>(cloud, msg->header.stamp);
    for (auto iter = sorted_clouds.begin(); iter != sorted_clouds.end();
         iter++) {
      ros::Time t;
      t.fromNSec(iter->first);
      LidarChunk<PointXYZIRT> lidar_chunk(t, iter->second);
      velodyne_lidar_aggregator_->Add(lidar_chunk);
    }
  } else if (params_.lidar_type == LidarType::OUSTER) {
    pcl::PointCloud<PointXYZITRRNR> cloud;
    beam::ROSToPCL(cloud, *msg);
    std::map<uint64_t, pcl::PointCloud<PointXYZITRRNR>> sorted_clouds =
        SeparateTimeStamps<PointXYZITRRNR>(cloud, msg->header.stamp);
    for (auto iter = sorted_clouds.begin(); iter != sorted_clouds.end();
         iter++) {
      ros::Time t;
      t.fromNSec(iter->first);
      LidarChunk<PointXYZITRRNR> lidar_chunk(t, iter->second);
      ouster_lidar_aggregator_->Add(lidar_chunk);
    }
  }

}

void LidarAggregation::ProcessTimeTrigger(const std_msgs::Time::ConstPtr& msg) {
  if (params_.lidar_type == LidarType::VELODYNE) {
    velodyne_lidar_aggregator_->Aggregate(msg->data);
    std::vector<LidarAggregate<PointXYZIRT>> aggregates =
        velodyne_lidar_aggregator_->Get();
    for (const auto& aggregate : aggregates) {
      sensor_msgs::PointCloud2 cloud_msg =
          beam::PCLToROS<PointXYZIRT>(aggregate.cloud, aggregate.time,
                                      extrinsics_.GetLidarFrameId(), counter_);
      aggregate_publisher_.publish(cloud_msg);
    }
  } else if (params_.lidar_type == LidarType::OUSTER) {
    ouster_lidar_aggregator_->Aggregate(msg->data);
    std::vector<LidarAggregate<PointXYZITRRNR>> aggregates =
        ouster_lidar_aggregator_->Get();
    for (const auto& aggregate : aggregates) {
      sensor_msgs::PointCloud2 cloud_msg = beam::PCLToROS<PointXYZITRRNR>(
          aggregate.cloud, aggregate.time, extrinsics_.GetLidarFrameId(),
          counter_);
      aggregate_publisher_.publish(cloud_msg);
    }
  }
  counter_++;
}

} // namespace bs_models::experimental
