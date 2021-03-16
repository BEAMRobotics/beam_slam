#include <beam_models/frame_initializers/odometry_frame_initializer.h>

#include <beam_utils/log.h>

namespace beam_models { namespace frame_initializers {

OdometryFrameInitializer::OdometryFrameInitializer(
    const std::string& topic, int queue_size,
    const std::string& sensor_frame_id, bool static_extrinsics,
    double poses_buffer_time) {
  poses_ = std::make_shared<tf2::BufferCore>(ros::Duration(poses_buffer_time));

  ros::NodeHandle n;
  odometry_subscriber_ = n.subscribe<nav_msgs::Odometry>(
      topic, queue_size,
      boost::bind(&OdometryFrameInitializer::OdometryCallback, this, _1));

  // set pose lookup params and set pose lookup to nullptr until first odom msg
  // comes in
  pose_lookup_params_.poses = poses_;
  pose_lookup_params_.sensor_frame = sensor_frame_id;
  pose_lookup_params_.static_extrinsics = static_extrinsics;
  pose_lookup_ = nullptr;
}

void OdometryFrameInitializer::OdometryCallback(
    const nav_msgs::OdometryConstPtr message) {
  // init pose lookup params if not already done
  if (pose_lookup_params_.baselink_frame.empty()) {
    pose_lookup_params_.baselink_frame = message->child_frame_id;
    if (pose_lookup_params_.baselink_frame.substr(0, 1) == "/") {
      pose_lookup_params_.baselink_frame.erase(0, 1);
    }
  }
  if (pose_lookup_params_.world_frame.empty()) {
    pose_lookup_params_.world_frame = message->header.frame_id;
    if (pose_lookup_params_.world_frame.substr(0, 1) == "/") {
      pose_lookup_params_.world_frame.erase(0, 1);
    }
  }
  if (pose_lookup_ == nullptr) {
    pose_lookup_ =
        std::make_unique<beam_common::PoseLookup>(pose_lookup_params_);
  }

  geometry_msgs::TransformStamped tf_stamped;
  tf_stamped.header = message->header;
  tf_stamped.child_frame_id = message->child_frame_id;
  tf_stamped.transform.translation.x = message->pose.pose.position.x;
  tf_stamped.transform.translation.y = message->pose.pose.position.y;
  tf_stamped.transform.translation.z = message->pose.pose.position.z;
  tf_stamped.transform.rotation = message->pose.pose.orientation;
  std::string authority{"odometry"};
  poses_->setTransform(tf_stamped, authority, false);
}

bool OdometryFrameInitializer::GetEstimatedPose(
    const ros::Time& time, Eigen::Matrix4d& T_WORLD_SENSOR) {
  if (pose_lookup_ == nullptr) { return false; }
  bool result = pose_lookup_->GetT_WORLD_SENSOR(T_WORLD_SENSOR, time);
  return result;
}

}} // namespace beam_models::frame_initializers