#pragma once

#include <bs_models/lidar_aggregators/lidar_aggregator_base.h>
#include <bs_models/frame_initializers/odometry_frame_initializer.h>

namespace bs_models {

/**
 * @brief This class takes a care of aggregating lidar chunks into a motion
 * compensated aggregate, where all poins in the aggregate come before the
 * aggregation_time. The aggregate contains all points expressed in the lidar
 * frame at the aggregation_time. Pose lookup and interpolation use a tf2 tree,
 * similar to the extrinsics which can be static or dynamic.
 *
 * NOTE: The is currently no way of checking that the lidar chunk times are not
 * prior to the tf tree start time. If you are getting lookup errors because you
 * area looking for a transform before the earliest available transform, change
 * the DEFAULT_CACHE_TIME in tf2::Buffercore
 */
class EndTimeLidarAggregator : public LidarAggregatorBase {
public:
  /**
   * @brief constructor
   * @param topic odometry topic to subscribe to
   * @param queue_size odometry subscriber queue size
   * @param poses_buffer_time length of time (in seconds) to store poses for
   * interpolation
   * @param sensor_frame_id_override frame ID attached to the sensor. If this is
   * set, it will override the sensor_frame in the odometry message
   * @param max_aggregate_duration maximum time duration to add points to a
   * single aggregate
   * @param lidar_type this will affect the type of point assumed
   */
  EndTimeLidarAggregator(
      const std::string& topic, int queue_size, int64_t poses_buffer_time,
      beam::LidarType lidar_type,
      const ros::Duration& max_aggregate_duration = ros::Duration(0.1),
      const std::string& sensor_frame_id_override = "");

  void Aggregate(const ros::Time& aggregation_time) override;

private:
  std::unique_ptr<bs_common::OdometryFrameInitializer> frame_intializer_;
  ros::Duration max_aggregate_duration_;
};

} // namespace bs_models