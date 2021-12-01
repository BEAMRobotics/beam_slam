#pragma once

#include <queue>

#include <beam_utils/pointclouds.h>

namespace bs_models {

template <typename PointT>

/**
 * @brief class to hold all lidar points that are collected at one time
 * instance. Points must be expressed in the lidar frame
 */
struct LidarChunk {
  std::shared_ptr<pcl::PointCloud<PointT>> cloud;
  ros::Time time;
  LidarChunk() { cloud = std::make_shared<pcl::PointCloud<PointT>>(); }
};

template <typename PointT>
struct LidarAggregate {
  std::shared_ptr<pcl::PointCloud<PointT>> cloud;
  ros::Time time;
  LidarAggregate() { cloud = std::make_shared<pcl::PointCloud<PointT>>(); }
};

/**
 * @brief This class takes a care of aggregating lidar chunks into a motion
 * compensated aggregate, where all poins in the aggregate come before the
 * aggregation_time. The aggregate contains all points expressed in the lidar
 * frame at the aggregation_time.
 */
template <typename PointT>
class LidarAggregatorBase {
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
      const std::string& sensor_frame_id_override = "") {
    max_aggregate_duration_ = max_aggregate_duration;
    frame_intializer_ = std::make_unique<bs_common::OdometryFrameInitializer>(
        topic, queue_size, poses_buffer_time, sensor_frame_id_override);
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

    while (!aggregation_times_.empty()) {
      const ros::time current_aggregate_time = aggregation_times_.front();

      // if the aggregation time is ahead of the first lidar chunk, then we
      // remove the aggregation time. This might cause some missed aggregates at
      // the start if there is a bit of delay in the lidar input, but shouldn't
      // have that big of an effect
      if (current_aggregate_time.toNSec() < lidar_chunks_.begin().first) {
        aggregation_times_.pop();
        continue;
      }

      // If aggregation time is ahead of the latest lidar chunk, then we return
      // and wait for more points to arrive
      if (current_aggregate_time.toNSec() > lidar_chunks_.rbegin().first) {
        return;
      }

      // if we get to here, then aggregation time is within the timestamps of
      // the points

      // get lidar pose at aggregation time
      Eigen::Matrix4d T_WORLD_LIDARAGG;
      if (!frame_intializer_->GetEstimatedPose(T_WORLD_LIDARAGG,
                                               current_aggregate_time,
                                               extrinsics_.GetLidarFrameId())) {
        BEAM_WARN(
            "Cannot get lidar pose at requested aggregation time, skipping.");
        return;
      }

      // iterate through all lidar chunks, transform to corrected position, then
      // add to final aggregate
      LidarAggregate<PointT> current_aggregate;
      current_aggregate.time = current_aggregate_time;
      while (!lidar_chunks_.empty()) {
        uint64_t current_chunk_time = lidar_chunks_.begin().first;

        // get lidar pose at current chunk time
        Eigen::Matrix4d T_WORLD_LIDARCHUNK;
        if (!frame_intializer_->GetEstimatedPose(
                T_WORLD_LIDARCHUNK, current_chunk_time,
                extrinsics_.GetLidarFrameId())) {
          BEAM_WARN("Cannot get lidar pose at lidar chunk time, skipping.");
          lidar_chunks_.erase(current_chunk_time);
          continue;
        }

        Eigen::Matrix4d T_LIDARCHUNK_LIDARAGG =
            beam::InvertTransform(T_WORLD_LIDARCHUNK) * T_WORLD_LIDARAGG;

        pcl::PointCloud<PointT> compensated_chunk;
        pcl::transformPointCloud(*(lidar_chunks_.begin().second.cloud),
                                 compensated_chunk, T_LIDARCHUNK_LIDARAGG);

        *(current_aggregate.cloud) += compensated_chunk;
        lidar_chunks_.erase(current_chunk_time);
      }
      finalized_aggregates_.push_back(current_aggregate);
    }
  }

private:
  std::map<uint64_t, LidarChunk<PointT>> lidar_chunks_; // t_nsec -> lidar chunk
  std::vector<LidarAggregate<PointT>> finalized_aggregates_;
  std::unique_ptr<bs_common::OdometryFrameInitializer> frame_intializer_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
  std::queue<ros::Time> aggregation_times_;

  // params
  ros::Duration max_aggregate_duration_;
};

} // namespace bs_models