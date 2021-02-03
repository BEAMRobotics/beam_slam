#include <lidar_aggregation/lidar_aggregators/lidar_aggregator_base.h>

#include <beam_utils/log.h>

void LidarAggregatorBase::Add(const LidarChunk& lidar_chunk) {
  if(lidar_chunks_.size() == 0){
    lidar_chunks_.push(lidar_chunk);
  } else if (lidar_chunk.time < lidar_chunks_.back().time) {
    BEAM_WARN("Lidar chunk time not increasing, ignoring.");
  } else {
    lidar_chunks_.push(lidar_chunk);
  }
}

std::vector<LidarAggregate> LidarAggregatorBase::Get() { 
  return std::move(finalized_aggregates_);
}