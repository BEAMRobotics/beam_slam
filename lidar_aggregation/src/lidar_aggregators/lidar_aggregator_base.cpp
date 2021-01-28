#include <lidar_aggregation/lidar_aggregators/lidar_aggregator_base.h>

#include <beam_utils/log.h>

void LidarAggregatorBase::Add(const LidarChunk& lidar_chunk) {
  if (lidar_chunk.time < lidar_chunks_.back().time) {
    BEAM_WARN("Lidar chunk time not increasing, ignoring.");
    return;
  }
  lidar_chunks_.push(lidar_chunk);
}

std::vector<LidarAggregate> LidarAggregatorBase::Get() {
  std::vector<LidarAggregate> output_vector;
  std::move(finalized_aggregates_.begin(), finalized_aggregates_.end(),
            output_vector.begin());
  return output_vector;
}