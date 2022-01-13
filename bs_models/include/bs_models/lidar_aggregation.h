#pragma once

#include <std_msgs/Time.h>
#include <fuse_core/async_sensor_model.h>
#include <fuse_core/throttled_callback.h>

#include <bs_models/lidar/lidar_aggregator.h>
#include <bs_parameters/models/lidar_aggregation_params.h>

namespace bs_models {

class LidarAggregation : public fuse_core::AsyncSensorModel {
public:
  SMART_PTR_DEFINITIONS(LidarAggregation);

  LidarAggregation();

  ~LidarAggregation() override = default;

private:
  void onStart() override;

  void onInit() override;

  void onStop() override;

  void ProcessPointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

  void ProcessTimeTrigger(const std_msgs::Time::ConstPtr& msg);

  template <typename PointT>
  std::map<uint64_t, pcl::PointCloud<PointT>>
      SeparateTimeStamps(const pcl::PointCloud<PointT>& cloud,
                         const ros::Time& stamp) {
    if (!point_stamps_checked_) {
      if (cloud.at(0).time == 0) {
        BEAM_WARN("Pointclouds do not have timestamped points. If scans are "
                  "output on a per-packet basis, this is okay, otherwise "
                  "motion compensation will not work.");
        per_point_timestamps_ = false;
      }
      point_stamps_checked_ = true;
    }

    std::map<uint64_t, pcl::PointCloud<PointT>> sorted_clouds;

    // if each point is not timestamped, then send back the whole cloud
    if (!per_point_timestamps_) {
      sorted_clouds.emplace(stamp.toNSec(), cloud);
      return sorted_clouds;
    }

    for (const PointT& p : cloud) {
      uint64_t t = stamp.toNSec();
      auto iter = sorted_clouds.find(t);
      if (iter == sorted_clouds.end()) {
        pcl::PointCloud<PointT> new_cloud;
        new_cloud.push_back(p);
        sorted_clouds.emplace(stamp.toNSec(), new_cloud);
      } else {
        iter->second.push_back(p);
      }
    }

    return sorted_clouds;
  }

  /** subscribe to lidar data */
  ros::Subscriber aggregation_time_subscriber_;
  ros::Subscriber pointcloud_subscriber_;

  /** Publishers */
  ros::Publisher aggregate_publisher_;

  /** callbacks */
  using ThrottledCallbackPC =
      fuse_core::ThrottledMessageCallback<sensor_msgs::PointCloud2>;
  ThrottledCallbackPC throttled_callback_pc_;

  using ThrottledCallbackTime =
      fuse_core::ThrottledMessageCallback<std_msgs::Time>;
  ThrottledCallbackTime throttled_callback_time_;

  bs_parameters::models::LidarAggregationParams params_;

  std::unique_ptr<LidarAggregator<PointXYZIRT>> velodyne_lidar_aggregator_;
  std::unique_ptr<LidarAggregator<PointXYZITRRNR>> ouster_lidar_aggregator_;

  bool per_point_timestamps_{true};
  bool point_stamps_checked_{false};
  int counter_{0};

  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  // params only tunable here
  double poses_buffer_time_{30};
  int odometry_subscriber_queue_size_{100};
  int lidar_subscriber_queue_size_{5};
  int aggregation_time_subscriber_queue_size_{5};
  int aggregate_publisher_queue_size_{10};
};

} // namespace bs_models
