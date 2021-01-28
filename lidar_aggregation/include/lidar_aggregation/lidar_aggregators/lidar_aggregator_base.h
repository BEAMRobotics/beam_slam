#pragma once

#include <queue>

#include <tf2/buffer_core.h>
#include <boost/make_shared.hpp>

#include <beam_utils/optional.h>
#include <beam_utils/pointclouds.h>


struct LidarChunk {
  PointCloudPtr cloud;
  ros::Time time;
  LidarChunk(){
    cloud = boost::make_shared<PointCloud>();
  }
};

struct LidarAggregate {
  PointCloudPtr cloud;
  ros::Time time;
  LidarAggregate(){
    cloud = boost::make_shared<PointCloud>();
  }
};

class LidarAggregatorBase {
public:
  /**
   * @brief Adds a new lidar chunk to the queue to be aggregated. It also checks that the timestamps are in increasing order, and ingores chunks that are not.
   * @param lidar_chunk set of lidar points to be added
   */
  void Add(const LidarChunk& lidar_chunk);

  std::vector<LidarAggregate> Get();

  /**
   * @brief Aggregates all lidar chunks added
   * @param aggregation_time time at which to motion compensation chunks to
   */
  virtual void Aggregate(const ros::Time& aggregation_time) = 0;

protected:
  std::queue<LidarChunk> lidar_chunks_;
  std::vector<LidarAggregate> finalized_aggregates_;
};
