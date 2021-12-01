#include <bs_models/lidar_aggregation.h>

#include <pluginlib/class_list_macros.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::LidarAggregation, fuse_core::SensorModel)

namespace bs_models {

LidarAggregation::LidarAggregation()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_callback_pc_(std::bind(&LidarAggregation::ProcessPointcloud,
                                       this, std::placeholders::_1)),
      throttled_callback_time_(std::bind(&LidarAggregation::ProcessTimeTrigger,
                                         this, std::placeholders::_1)) {}

void LidarAggregation::onInit() {
  params_.loadFromROS(private_node_handle_);

  lidar_aggregator_ = std::make_unique<EndTimeLidarAggregator>(
      params_.odometry_topic, odometry_queue_size_, poses_buffer_time_,
      ros::Duration(params_.max_aggregation_time_seconds),
      params_.sensor_frame_id_override, params_.lidar_type);
}

void LidarAggregation::onStart() {
  pointcloud_subscriber_ = node_handle_.subscribe<sensor_msgs::PointCloud2>(
      ros::names::resolve(params_.pointcloud_topic),
      lidar_subscriber_queue_size_, &ThrottledCallbackPC::callback,
      &throttled_callback_pc_, ros::TransportHints().tcpNoDelay(false));

  aggregation_time_subscriber_ = node_handle_.subscribe<std_msgs::Time>(
      ros::names::resolve(params_.aggregation_time_topic),
      aggregation_time_subscriber_queue_size_, &ThrottledCallbackTime::callback,
      &throttled_callback_time_, ros::TransportHints().tcpNoDelay(false));

  aggregate_publisher_ =
      private_node_handle_.advertise<sensor_msgs::PointCloud2>(
          params_.aggregate_topic, aggregate_publisher_queue_size_);
}

void LidarAggregation::onStop() {
  aggregation_time_subscriber_.shutdown();
  pointcloud_subscriber_.shutdown();
  odometry_subscriber_.shutdown();
}

void LidarAggregation::ProcessPointcloud(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (params_.lidar_type == beam::LidarType::VELODYNE) {
    pcl::PointCloud<PointXYZIRT> cloud;
    beam::ROSToPCL(cloud, *msg);
    std::map<uint64_t, pcl::PointCloud<PointXYZIRT>> sorted_clouds =
        SeparateTimeStamps<PointXYZIRT>(cloud, msg.stamp);
    for (auto iter = sorted_clouds.begin(); iter != sorted_clouds.end();
         iter++) {
      LidarChunk lidar_chunk<PointXYZIRT>();
      lidar_chunk.stamp.fromNSec(iter->first);
      *(lidar_chunk.cloud) = iter->second;
      lidar_aggregator_->Add(lidar_chunk);
    }
  } else if (params_.lidar_type == beam::LidarType::OUSTER) {
    pcl::PointCloud<PointXYZITRRNR> cloud;
    beam::ROSToPCL(cloud, *msg);
    std::map<uint64_t, pcl::PointCloud<PointXYZITRRNR>> sorted_clouds =
        SeparateTimeStamps<PointXYZITRRNR>(cloud, msg.stamp);
    for (auto iter = sorted_clouds.begin(); iter != sorted_clouds.end();
         iter++) {
      LidarChunk lidar_chunk<PointXYZITRRNR>();
      lidar_chunk.stamp.fromNSec(iter->first);
      *(lidar_chunk.cloud) = iter->second;
      lidar_aggregator_->Add(lidar_chunk);
    }
  } else {
    BEAM_ERROR(
        "Invalid lidar type param. Lidar type may not be implemented yet.");
    throw std::invalid_argument{"Invalid lidar type."};
  }
}

void LidarAggregation::ProcessTimeTrigger(const std_msgs::Time::ConstPtr& msg) {
  lidar_aggregator_->Aggregate(message->data);
  if (params_.lidar_type == beam::LidarType::VELODYNE) {
    std::vector<LidarAggregate<PointXYZIRT>> aggregates =
        lidar_aggregator_->Get();
    for (const auto& cloud : aggregates) {
      sensor_msgs::PointCloud2 cloud_msg = beam::PCLToROS(cloud_msg, cloud);
      aggregate_publisher_.publish(cloud_msg);
    }
  } else if (params_.lidar_type == beam::LidarType::OUSTER) {
    std::vector<LidarAggregate<PointXYZITRRNR>> aggregates =
        lidar_aggregator_->Get();
    for (const auto& cloud : aggregates) {
      sensor_msgs::PointCloud2 cloud_msg = beam::PCLToROS(cloud_msg, cloud);
      aggregate_publisher_.publish(cloud_msg);
    }
  } else {
    BEAM_ERROR(
        "Invalid lidar type param. Lidar type may not be implemented yet.");
    throw std::invalid_argument{"Invalid lidar type."};
  }
}

} // namespace bs_models
