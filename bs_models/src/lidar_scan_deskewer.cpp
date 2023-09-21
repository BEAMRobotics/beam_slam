#include <bs_models/lidar_scan_deskewer.h>

#include <pcl/common/transforms.h>
#include <pluginlib/class_list_macros.h>

#include <beam_utils/se3.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::LidarScanDeskewer, fuse_core::SensorModel)

namespace bs_models {

LidarScanDeskewer::LidarScanDeskewer()
    : fuse_core::AsyncSensorModel(1),
      throttled_callback_pc_(std::bind(&LidarScanDeskewer::ProcessPointcloud,
                                       this, std::placeholders::_1)) {}

void LidarScanDeskewer::onInit() {
  ROS_DEBUG("Initialzing LidarScanDeskewer");
  params_.loadFromROS(private_node_handle_);
  ROS_DEBUG("Loaded params");

  frame_initializer_ =
      std::make_unique<bs_models::FrameInitializer>(
          params_.frame_initializer_config);
}

void LidarScanDeskewer::onStart() {
  ROS_DEBUG("Starting subscribers");
  pointcloud_subscriber_ =
      private_node_handle_.subscribe<sensor_msgs::PointCloud2>(
          ros::names::resolve(params_.input_topic),
          pointcloud_subscriber_queue_size_, &ThrottledCallbackPC::callback,
          &throttled_callback_pc_, ros::TransportHints().tcpNoDelay(false));

  ROS_DEBUG("Starting publisher");
  pointcloud_publisher_ =
      private_node_handle_.advertise<sensor_msgs::PointCloud2>(
          "points_undistorted", pointcloud_publisher_queue_size_);
  ROS_DEBUG("Done start routine");
}

void LidarScanDeskewer::onStop() {
  ROS_DEBUG("Shutting down publishers and subscribers");
  pointcloud_subscriber_.shutdown();
  ROS_DEBUG("Done shutdown routine");
}

void LidarScanDeskewer::ProcessPointcloud(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (params_.lidar_type == LidarType::VELODYNE) {
    ROS_DEBUG("Processing Velodyne poincloud message");
    pcl::PointCloud<PointXYZIRT> cloud;
    beam::ROSToPCL(cloud, *msg);
    queue_velodyne_.emplace(VelodyneCloudWithStamp{msg->header.stamp, cloud});
    DeskewAndPublishVelodyneQueue();
  } else if (params_.lidar_type == LidarType::OUSTER) {
    ROS_DEBUG("Processing Ouster poincloud message");
    pcl::PointCloud<PointXYZITRRNR> cloud;
    beam::ROSToPCL(cloud, *msg);
    queue_ouster_.emplace(OusterCloudWithStamp{msg->header.stamp, cloud});
    DeskewAndPublishOusterQueue();
  } else {
    throw std::runtime_error{
        "function not yet implemented for input lidar type"};
  }
}

void LidarScanDeskewer::DeskewAndPublishVelodyneQueue() {
  while (!queue_velodyne_.empty()) {
    const ros::Time& cloud_stamp = queue_velodyne_.front().first;
    const pcl::PointCloud<PointXYZIRT>& cloud = queue_velodyne_.front().second;

    // get pose of the cloud stamp (this may or may not be the first point)
    Eigen::Matrix4d T_World_Lidar0;
    if (!frame_initializer_->GetPose(T_World_Lidar0, cloud_stamp,
                                     extrinsics_.GetLidarFrameId())) {
      break;
    }
    Eigen::Matrix4f T_Lidar0_World =
        beam::InvertTransform(T_World_Lidar0).cast<float>();

    pcl::PointCloud<PointXYZIRT> cloud_deskewed;
    for (const auto& p : cloud) {
      ros::Time pt = cloud_stamp + ros::Duration(p.time);
      Eigen::Matrix4d T_World_LidarN;
      if (!frame_initializer_->GetPose(T_World_LidarN, pt,
                                       extrinsics_.GetLidarFrameId())) {
        // if any point doesn't return a valid result, then skip
        break;
      }
      Eigen::Affine3f T_Lidar0_LidarN(T_Lidar0_World *
                                      T_World_LidarN.cast<float>());
      PointXYZIRT p_deskewed =
          pcl::transformPoint<PointXYZIRT>(p, T_Lidar0_LidarN);
      cloud_deskewed.push_back(p_deskewed);
    }

    sensor_msgs::PointCloud2 cloud_msg = beam::PCLToROS<PointXYZIRT>(
        cloud_deskewed, cloud_stamp, extrinsics_.GetLidarFrameId(), counter_++);
    pointcloud_publisher_.publish(cloud_msg);
    queue_velodyne_.pop();
  }

  // clear buffer overflow
  while (queue_velodyne_.size() > params_.scan_buffer_size) {
    queue_velodyne_.pop();
  }
}

void LidarScanDeskewer::DeskewAndPublishOusterQueue() {
  while (!queue_ouster_.empty()) {
    const ros::Time& cloud_stamp = queue_ouster_.front().first;
    const pcl::PointCloud<PointXYZITRRNR>& cloud = queue_ouster_.front().second;

    // get pose of the cloud stamp (this may or may not be the first point)
    Eigen::Matrix4d T_World_Lidar0;
    if (!frame_initializer_->GetPose(T_World_Lidar0, cloud_stamp,
                                     extrinsics_.GetLidarFrameId())) {
      break;
    }
    Eigen::Matrix4f T_Lidar0_World =
        beam::InvertTransform(T_World_Lidar0).cast<float>();

    pcl::PointCloud<PointXYZITRRNR> cloud_deskewed;
    for (const auto& p : cloud) {
      ros::Time pt = cloud_stamp + ros::Duration(p.time);
      Eigen::Matrix4d T_World_LidarN;
      if (!frame_initializer_->GetPose(T_World_LidarN, pt,
                                       extrinsics_.GetLidarFrameId())) {
        // if any point doesn't return a valid result, then skip
        break;
      }
      Eigen::Affine3f T_Lidar0_LidarN(T_Lidar0_World *
                                      T_World_LidarN.cast<float>());
      PointXYZITRRNR p_deskewed =
          pcl::transformPoint<PointXYZITRRNR>(p, T_Lidar0_LidarN);
      cloud_deskewed.push_back(p_deskewed);
    }

    sensor_msgs::PointCloud2 cloud_msg = beam::PCLToROS<PointXYZITRRNR>(
        cloud_deskewed, cloud_stamp, extrinsics_.GetLidarFrameId(), counter_++);
    pointcloud_publisher_.publish(cloud_msg);
    queue_ouster_.pop();
  }

  // clear buffer overflow
  while (queue_ouster_.size() > params_.scan_buffer_size) {
    queue_ouster_.pop();
  }
}

} // namespace bs_models
