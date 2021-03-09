#pragma once

#include <beam_utils/optional.h>
#include <beam_utils/pointclouds.h>

#include <lidar_aggregation/lidar_aggregators/lidar_aggregator_base.h>

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
   * @param poses pointer to tf tree of poses (T_WORLD_BASELINK)
   * @param extrinsics pointer to tf tree of extrinsics (T_BASELINK_LIDAR). If
   * extrinsics are static, this will be one transform.
   * @param first_pose_time time associated with the first pose. This is stored as a const ref so it can be changed outside thie class when the first pose is added
   * @param baselink_frame moving frame in poses. This is stored as a const ref so it can be changed outside thie class when the first pose is added
   * @param lidar_frame source (from) frame in extrinsics. This is usually the
   * lidar frame. Aggregates will remain in this frame. This is stored as a const ref so it can be changed outside thie class when the first pose is added
   * @param baselink_frame fixed frame in poses. This is stored as a const ref so it can be changed outside thie class when the first pose is added
   * @param static_extrinsics set to true if extrinsics do not change with time
   * @param clear_queue_on_update set to true to clear queue of lidar chunks
   * each time Aggregate() is called
   */
  EndTimeLidarAggregator(
      const std::shared_ptr<tf2::BufferCore> poses,
      const std::shared_ptr<tf2::BufferCore> extrinsics,
      const ros::Time& first_pose_time,
      const std::string& baselink_frame, const std::string& lidar_frame,
      const std::string& world_frame,
      const ros::Duration& max_aggregate_duration = ros::Duration(5),
      bool static_extrinsics = true, bool clear_queue_on_update = false);

  void Aggregate(const ros::Time& aggregation_time) override;

private:
  Eigen::Matrix4d LookupT_BASELINK_LIDAR(const ros::Time& time);

  Eigen::Matrix4d LookupT_WORLD_BASELINK(const ros::Time& time);

  ros::Time GetEarliestPoseTime();

  std::shared_ptr<tf2::BufferCore> poses_;
  std::shared_ptr<tf2::BufferCore> extrinsics_;
  const std::string& baselink_frame_;
  const std::string& lidar_frame_;
  const std::string& world_frame_;
  const ros::Time& first_pose_time_;
  ros::Duration max_aggregate_duration_;
  bool static_extrinsics_;
  bool clear_queue_on_update_;
};
