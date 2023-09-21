#pragma once

#include <queue>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/throttled_callback.h>

#include <bs_models/frame_initializers/frame_initializer.h>
#include <bs_parameters/models/lidar_scan_deskewer_params.h>

namespace bs_models {

// using SortedPacketsVelodyne = std::map<int64_t, pcl::PointCloud<PointXYZIRT>>;
// using SortedPacketsOuster = std::map<int64_t, pcl::PointCloud<PointXYZITRRNR>>;

class LidarScanDeskewer : public fuse_core::AsyncSensorModel {
public:
  FUSE_SMART_PTR_DEFINITIONS(LidarScanDeskewer);

  LidarScanDeskewer();

  ~LidarScanDeskewer() override = default;

private:
  void onStart() override;

  void onInit() override;

  void onStop() override;

  void ProcessPointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

  void DeskewAndPublishVelodyneQueue();

  void DeskewAndPublishOusterQueue();

  /** subscribe to lidar data */
  ros::Subscriber pointcloud_subscriber_;

  /** Publishers */
  ros::Publisher pointcloud_publisher_;

  /** callbacks */
  using ThrottledCallbackPC =
      fuse_core::ThrottledMessageCallback<sensor_msgs::PointCloud2>;
  ThrottledCallbackPC throttled_callback_pc_;

  bs_parameters::models::LidarScanDeskewerParams params_;

  int counter_{0};

  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
  std::unique_ptr<bs_models::FrameInitializer> frame_initializer_;

  using VelodyneCloudWithStamp = std::pair<ros::Time, pcl::PointCloud<PointXYZIRT>>;
  using OusterCloudWithStamp = std::pair<ros::Time, pcl::PointCloud<PointXYZITRRNR>>;

  std::queue<VelodyneCloudWithStamp> queue_velodyne_;
  std::queue<OusterCloudWithStamp> queue_ouster_;

  // params only tunable here
  int pointcloud_subscriber_queue_size_{5};
  int pointcloud_publisher_queue_size_{10};
};

} // namespace bs_models
