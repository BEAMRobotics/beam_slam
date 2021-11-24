#pragma once

#include <queue>

#include <beam_utils/pointclouds.h>

namespace bs_models {

template <typename PointT>
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

template <typename PointT>
class LidarAggregatorBase {
public:
  /**
   * @brief Adds a new lidar chunk to the queue to be aggregated. It also checks
   * that the timestamps are in increasing order, and ingores chunks that are
   * not.
   * @param lidar_chunk set of lidar points to be added
   */
  void Add(const LidarChunk<PointT>& lidar_chunk) {
    if (lidar_chunks_.size() == 0) {
      lidar_chunks_.push(lidar_chunk);
    } else if (lidar_chunk.time < lidar_chunks_.back().time) {
      BEAM_WARN("Lidar chunk time not increasing, ignoring.");
    } else {
      lidar_chunks_.push(lidar_chunk);
    }
  }

  std::vector<LidarAggregate<PointT>> Get() {
    return std::move(finalized_aggregates_);
  }

  /**
   * @brief Aggregates all lidar chunks added
   * @param aggregation_time time at which to motion compensation chunks to
   */
  virtual void Aggregate(const ros::Time& aggregation_time) = 0;

protected:
  std::queue<LidarChunk<PointT>> lidar_chunks_;
  std::vector<LidarAggregate<PointT>> finalized_aggregates_;
};

} // namespace bs_models