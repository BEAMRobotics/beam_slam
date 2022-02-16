#pragma once

#include <queue>

#include <pcl/common/transforms.h>

#include <beam_utils/math.h>
#include <beam_utils/pointclouds.h>

#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/frame_initializers/odometry_frame_initializer.h>

namespace bs_models {

/**
 * @brief class to hold all lidar points that are collected at one time
 * instance. Points must be expressed in the lidar frame
 */
template <typename PointT>
struct LidarChunk {
  pcl::PointCloud<PointT> cloud;
  ros::Time time;
  LidarChunk(const ros::Time& t, const pcl::PointCloud<PointT>& c)
      : time(t), cloud(c) {}
};

template <typename PointT>
struct LidarAggregate {
  pcl::PointCloud<PointT> cloud;
  ros::Time time;
  LidarAggregate(const ros::Time& t) : time(t) {}
};

/**
 * @brief This class takes a care of aggregating lidar chunks into a motion
 * compensated aggregate, where all poins in the aggregate come before the
 * aggregation_time. The aggregate contains all points expressed in the lidar
 * frame at the aggregation_time.
 */
template <typename PointT>
class LidarAggregator {
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
   */
  LidarAggregator(
      const std::string& topic, int queue_size, int64_t poses_buffer_time,
      const ros::Duration& max_aggregate_duration = ros::Duration(0.1),
      const std::string& sensor_frame_id_override = "",
      const Eigen::Matrix4d& T_ORIGINAL_OVERRIDE =
          Eigen::Matrix4d::Identity()) {
    max_aggregate_duration_ = max_aggregate_duration;
    frame_intializer_ =
        std::make_unique<frame_initializers::OdometryFrameInitializer>(
            topic, queue_size, poses_buffer_time, sensor_frame_id_override,
            T_ORIGINAL_OVERRIDE);
  }

  /**
   * @brief Adds a new lidar chunk to the sorted map of chunks to be aggregated.
   * @param lidar_chunk set of lidar points to be added
   */
  void Add(const LidarChunk<PointT>& lidar_chunk) {
    lidar_chunks_.emplace(lidar_chunk.time.toNSec(), lidar_chunk);
  }

  std::vector<LidarAggregate<PointT>> Get() {
    return std::move(finalized_aggregates_);
  }

  /**
   * @brief Aggregates all lidar chunks added
   * @param aggregation_time time at which to motion compensation chunks to
   */
  void Aggregate(const ros::Time& aggregation_time) {
    aggregation_times_.push(aggregation_time);

    // check aggregates have been added
    if (lidar_chunks_.size() == 0) { return; }

    // continue as long as aggregation times and lidar chunks are not empty
    while (!aggregation_times_.empty() && !lidar_chunks_.empty()) {
      const ros::Time current_aggregate_time = aggregation_times_.front();

      // if the aggregation time is ahead of the first lidar chunk, then we
      // remove the aggregation time. This might cause some missed aggregates at
      // the start if there is a bit of delay in the lidar input, but shouldn't
      // have that big of an effect
      if (current_aggregate_time.toNSec() < lidar_chunks_.begin()->first) {
        aggregation_times_.pop();
        continue;
      }

      // If aggregation time is after the latest lidar chunk, then we return
      // and wait for more points to arrive
      if (current_aggregate_time.toNSec() > lidar_chunks_.rbegin()->first) {
        return;
      }

      // if we get to here, then aggregation time is within the timestamps of
      // the points

      // get lidar pose at aggregation time
      Eigen::Matrix4d T_WORLD_LIDARAGG;
      std::string error_msg;
      if (!frame_intializer_->GetEstimatedPose(
              T_WORLD_LIDARAGG, current_aggregate_time,
              extrinsics_.GetLidarFrameId(), error_msg)) {
        ROS_DEBUG("Cannot get lidar pose at requested aggregation time, "
                  "skipping. Reason: %s",
                  error_msg.c_str());
        return;
      }

      // iterate through all lidar chunks, transform to corrected position, then
      // add to final aggregate
      LidarAggregate<PointT> current_aggregate(current_aggregate_time);

      while (!lidar_chunks_.empty()) {
        const auto& current_chunk = lidar_chunks_.begin();
        uint64_t current_chunk_time = lidar_chunks_.begin()->first;

        // get lidar pose at current chunk time
        Eigen::Matrix4d T_WORLD_LIDARCHUNK;
        std::string error_msg;
        if (!frame_intializer_->GetEstimatedPose(
                T_WORLD_LIDARCHUNK, current_chunk->second.time,
                extrinsics_.GetLidarFrameId(), error_msg)) {
          ROS_DEBUG(
              "Cannot get lidar pose at lidar chunk time, skipping. Reason: {}",
              error_msg.c_str());
          lidar_chunks_.erase(current_chunk->first);
          continue;
        }

        Eigen::Matrix4d T_LIDARAGG_LIDARCHUNK =
            beam::InvertTransform(T_WORLD_LIDARAGG) * T_WORLD_LIDARCHUNK;

        pcl::PointCloud<PointT> compensated_chunk;
        pcl::transformPointCloud(current_chunk->second.cloud, compensated_chunk,
                                 T_LIDARAGG_LIDARCHUNK);

        current_aggregate.cloud += compensated_chunk;
        lidar_chunks_.erase(current_chunk_time);
      }
      finalized_aggregates_.push_back(current_aggregate);
    }
  }

private:
  std::map<uint64_t, LidarChunk<PointT>> lidar_chunks_; // t_nsec -> lidar chunk
  std::vector<LidarAggregate<PointT>> finalized_aggregates_;
  std::unique_ptr<frame_initializers::OdometryFrameInitializer>
      frame_intializer_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
  std::queue<ros::Time> aggregation_times_;

  // params
  ros::Duration max_aggregate_duration_;
};
} // namespace bs_models
