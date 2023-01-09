#include <bs_models/lidar_feature_extractor.h>

#include <pluginlib/class_list_macros.h>
#include <ros/names.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::LidarFeatureExtractor, fuse_core::SensorModel)

namespace bs_models {

LidarFeatureExtractor::LidarFeatureExtractor()
    : fuse_core::AsyncSensorModel(1),
      throttled_callback_pc_(
          std::bind(&LidarFeatureExtractor::ProcessPointcloud, this,
                    std::placeholders::_1)) {}

void LidarFeatureExtractor::onInit() {
  ROS_DEBUG("Initialzing LidarFeatureExtractor");
  params_.loadFromROS(private_node_handle_);
  ROS_DEBUG("Loaded params");

  // TODO

  ROS_DEBUG("Done LidarFeatureExtractor initialization");
}

void LidarFeatureExtractor::onStart() {
  ROS_DEBUG("Starting subscribers");
  pointcloud_subscriber_ =
      private_node_handle_.subscribe<sensor_msgs::PointCloud2>(
          ros::names::resolve(params_.pointcloud_topic), subscriber_queue_size_,
          &ThrottledCallbackPC::callback, &throttled_callback_pc_,
          ros::TransportHints().tcpNoDelay(false));

  ROS_DEBUG("Starting publisher");
  aggregate_publisher_ =
      private_node_handle_.advertise<sensor_msgs::PointCloud2>(
          "lidar_features", publisher_queue_size_);
  ROS_DEBUG("Done start routine");
}

void LidarFeatureExtractor::onStop() {
  ROS_DEBUG("Shutting down publishers and subscribers");
  pointcloud_subscriber_.shutdown();
  ROS_DEBUG("Done shutdown routine");
}

void LidarFeatureExtractor::ProcessPointcloud(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (params_.lidar_type == LidarType::VELODYNE) {
    ROS_DEBUG("Processing Velodyne poincloud message");
    pcl::PointCloud<PointXYZIRT> cloud;
    beam::ROSToPCL(cloud, *msg);

    // todo

  } else if (params_.lidar_type == LidarType::OUSTER) {
    ROS_DEBUG("Processing Ouster poincloud message");
    pcl::PointCloud<PointXYZITRRNR> cloud;
    beam::ROSToPCL(cloud, *msg);

    // todo
  }
}

} // namespace bs_models
