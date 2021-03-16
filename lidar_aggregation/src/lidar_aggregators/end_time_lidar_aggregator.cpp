#include <lidar_aggregation/lidar_aggregators/end_time_lidar_aggregator.h>

#include <ros/console.h>

#include <beam_utils/math.h>

EndTimeLidarAggregator::EndTimeLidarAggregator(
    const std::shared_ptr<tf2::BufferCore> poses,
    const std::shared_ptr<tf2::BufferCore> extrinsics,
    const ros::Time& first_pose_time, const std::string& baselink_frame,
    const std::string& lidar_frame, const std::string& world_frame,
    const ros::Duration& max_aggregate_duration, bool static_extrinsics,
    bool clear_queue_on_update)
    : poses_(poses),
      extrinsics_(extrinsics),
      first_pose_time_(first_pose_time),
      baselink_frame_(baselink_frame),
      lidar_frame_(lidar_frame),
      world_frame_(world_frame),
      max_aggregate_duration_(max_aggregate_duration),
      static_extrinsics_(static_extrinsics),
      clear_queue_on_update_(clear_queue_on_update) {}

void EndTimeLidarAggregator::Aggregate(const ros::Time& aggregation_time) {
  // check aggregates have been added
  if (lidar_chunks_.size() == 0) { return; }

  // get extrinsics if static
  Eigen::Matrix4d T_BASELINK_LIDAR;
  Eigen::Matrix4d T_LIDAR_BASELINK;
  if (static_extrinsics_) {
    bool success = true;
    T_BASELINK_LIDAR = LookupT_BASELINK_LIDAR(ros::Time(0), success);
    if (!success) {
      BEAM_WARN("Cannot perform aggregation, skipping trigger at time: %.4f",
                aggregation_time.toSec());
      return;
    }
    T_LIDAR_BASELINK = beam::InvertTransform(T_BASELINK_LIDAR);
  }

  // get pose for aggregate time
  bool success = true;
  Eigen::Matrix4d T_WORLD_BASELINK_AGGT =
      LookupT_WORLD_BASELINK(aggregation_time, success);
  if (!success) {
    BEAM_WARN("Cannot perform aggregation, skipping trigger at time: %.4f",
              aggregation_time.toSec());
    return;
  }

  Eigen::Matrix4d T_BASELINK_AGGT_WORLD =
      beam::InvertTransform(T_WORLD_BASELINK_AGGT);

  // iteratre through stored lidar chunks and add to current aggregate until
  // chunk time exceeds pose or extrinsic time in which case we break
  LidarAggregate current_aggregate;
  current_aggregate.time = aggregation_time;
  while (!lidar_chunks_.empty()) {
    LidarChunk current_chunk = lidar_chunks_.front();

    // check current chunk time is after first avaibable pose, if not then pop
    if (first_pose_time_ > current_chunk.time) {
      ROS_DEBUG("Pose time for current lidar chunk not available, skipping.");
      lidar_chunks_.pop();
      continue;
    }

    // check current chunk time prior to last available pose
    ros::Time latest_pose_available;
    std::string error_string;
    poses_->_getLatestCommonTime(poses_->_lookupFrameNumber(world_frame_),
                                 poses_->_lookupFrameNumber(baselink_frame_),
                                 latest_pose_available, &error_string);

    if (current_chunk.time > latest_pose_available) {
      ROS_DEBUG("Chunk time is past the latest pose time, stopping "
                "aggregation.");
      break;
    }

    // check extrinsic time is available if not static, then lookup extrinsic
    if (!static_extrinsics_) {
      ros::Time latest_extrinsic_available;
      extrinsics_->_getLatestCommonTime(
          extrinsics_->_lookupFrameNumber(lidar_frame_),
          extrinsics_->_lookupFrameNumber(baselink_frame_),
          latest_extrinsic_available, &error_string);

      if (current_chunk.time > latest_extrinsic_available) { break; }
      bool success = true;
      T_BASELINK_LIDAR = LookupT_BASELINK_LIDAR(current_chunk.time, success);

      if (!success) {
        BEAM_WARN("Cannot lookup extrinsics, skipping chunk at time: %.4f",
                  current_chunk.time.toSec());
        lidar_chunks_.pop();
        continue;
      }

      T_LIDAR_BASELINK = beam::InvertTransform(T_BASELINK_LIDAR);
    }

    // check that chunk_time is less than aggregation time
    if (current_chunk.time > aggregation_time) {
      ROS_DEBUG("Chunk time is past the aggregation time, stopping "
                "aggregation.");
      break;
    }

    lidar_chunks_.pop();

    // check duration of aggregate
    if (aggregation_time - current_chunk.time > max_aggregate_duration_) {
      ROS_DEBUG("Chunk time is outside aggregation window, skipping chunk.");
      continue;
    }

    // transform all points from chunk and add to aggregate
    for (PointCloud::iterator it = current_chunk.cloud->begin();
         it != current_chunk.cloud->end(); it++) {
      bool success = true;
      Eigen::Matrix4d T_WORLD_BASELINK_CHUNKT =
          LookupT_WORLD_BASELINK(current_chunk.time, success);
      if (!success) {
        BEAM_WARN("Cannot transform chunk, skipping chunk with timestamp: %.4f",
                  current_chunk.time.toSec());
        continue;
      }
      Eigen::Vector4d P_LIDAR_CHUNKT{it->x, it->y, it->z, 1};
      Eigen::Vector4d P_LIDAR_AGGT = T_LIDAR_BASELINK * T_BASELINK_AGGT_WORLD *
                                     T_WORLD_BASELINK_CHUNKT *
                                     T_BASELINK_LIDAR * P_LIDAR_CHUNKT;
      current_aggregate.cloud->push_back(
          pcl::PointXYZ(P_LIDAR_AGGT[0], P_LIDAR_AGGT[1], P_LIDAR_AGGT[2]));
    }
  }

  // add to finalized aggregates
  finalized_aggregates_.push_back(current_aggregate);

  if (clear_queue_on_update_) { lidar_chunks_ = std::queue<LidarChunk>(); }
  return;
}

ros::Time EndTimeLidarAggregator::GetEarliestPoseTime() {
  std::string error_msg;
  bool can_transform = poses_->canTransform(world_frame_, baselink_frame_,
                                            ros::Time(0), &error_msg);

  if (!can_transform) {
    ROS_ERROR("Cannot lookup T_WORLD_BASELINK: %s.\n Stored frames: %s",
              error_msg.c_str(), poses_->_allFramesAsDot().c_str());
    throw std::runtime_error{"Cannot lookup T_WORLD_BASELINK"};
  }

  geometry_msgs::TransformStamped T_WORLD_BASELINK =
      poses_->lookupTransform(world_frame_, baselink_frame_, ros::Time(0));
  return T_WORLD_BASELINK.header.stamp;
}

Eigen::Matrix4d
    EndTimeLidarAggregator::LookupT_WORLD_BASELINK(const ros::Time& time,
                                                   bool& success) {
  std::string error_msg;
  bool can_transform =
      poses_->canTransform(world_frame_, baselink_frame_, time, &error_msg);

  if (!can_transform) {
    ROS_WARN("Cannot lookup T_WORLD_BASELINK: %s.\n Stored frames: %s",
             error_msg.c_str(), poses_->_allFramesAsDot().c_str());
    success = false;
    return Eigen::Matrix4d::Identity();
  }

  geometry_msgs::TransformStamped T_WORLD_BASELINK =
      poses_->lookupTransform(world_frame_, baselink_frame_, time);

  // convert to eigen
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3) = T_WORLD_BASELINK.transform.translation.x;
  T(1, 3) = T_WORLD_BASELINK.transform.translation.y;
  T(2, 3) = T_WORLD_BASELINK.transform.translation.z;
  T(2, 3) = T_WORLD_BASELINK.transform.translation.z;
  Eigen::Quaternionf q;
  q.x() = T_WORLD_BASELINK.transform.rotation.x;
  q.y() = T_WORLD_BASELINK.transform.rotation.y;
  q.z() = T_WORLD_BASELINK.transform.rotation.z;
  q.w() = T_WORLD_BASELINK.transform.rotation.w;
  T.block(0, 0, 3, 3) = q.toRotationMatrix();
  success = true;
  return T.cast<double>();
}

Eigen::Matrix4d
    EndTimeLidarAggregator::LookupT_BASELINK_LIDAR(const ros::Time& time,
                                                   bool& success) {
  std::string error_msg;
  bool can_transform = extrinsics_->canTransform(lidar_frame_, baselink_frame_,
                                                 time, &error_msg);

  if (!can_transform) {
    BEAM_WARN("Cannot lookup T_LIDAR_BASELINK: {}", error_msg);
    success = false;
    return Eigen::Matrix4d::Identity();
  }

  geometry_msgs::TransformStamped T_LIDAR_BASELINK =
      extrinsics_->lookupTransform(lidar_frame_, baselink_frame_, time);

  // convert to eigen
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3) = T_LIDAR_BASELINK.transform.translation.x;
  T(1, 3) = T_LIDAR_BASELINK.transform.translation.y;
  T(2, 3) = T_LIDAR_BASELINK.transform.translation.z;
  T(2, 3) = T_LIDAR_BASELINK.transform.translation.z;
  Eigen::Quaternionf q;
  q.x() = T_LIDAR_BASELINK.transform.rotation.x;
  q.y() = T_LIDAR_BASELINK.transform.rotation.y;
  q.z() = T_LIDAR_BASELINK.transform.rotation.z;
  q.w() = T_LIDAR_BASELINK.transform.rotation.w;
  T.block(0, 0, 3, 3) = q.toRotationMatrix();

  success = true;
  return T.cast<double>();
}