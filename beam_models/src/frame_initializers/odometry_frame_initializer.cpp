#include <beam_models/frame_initializers/odometry_frame_initializer.h>

#include <beam_utils/log.h>

namespace beam_models {
namespace frame_initializers {

OdometryFrameInitializer::OdometryFrameInitializer(
    const std::string& topic, int queue_size, int64_t poses_buffer_time,
    const std::string& sensor_frame_id)
    : FrameInitializerBase(sensor_frame_id) {
  poses_ = std::make_shared<tf2::BufferCore>(ros::Duration(poses_buffer_time));
  pose_lookup_.SetPoses(poses_);

  ros::NodeHandle n;
  odometry_subscriber_ = n.subscribe<nav_msgs::Odometry>(
      topic, queue_size,
      boost::bind(&OdometryFrameInitializer::OdometryCallback, this, _1));
}

void OdometryFrameInitializer::CheckOdometryFrameIDs(
    const nav_msgs::OdometryConstPtr message) {
  std::string parent_frame_id = message->header.frame_id;
  std::string child_frame_id = message->child_frame_id;

  if (parent_frame_id.find(pose_lookup_.GetWorldFrameID()) ==
          std::string::npos ||
      child_frame_id.find(pose_lookup_.GetBaselinkFrameID()) ==
          std::string::npos) {
    BEAM_WARN(
        "World frame and baselink frames do not match parent and child frames "
        "in odometry messages, respectively.");
  }
  check_world_baselink_frames_ = false;
}

void OdometryFrameInitializer::OdometryCallback(
    const nav_msgs::OdometryConstPtr message) {
  if (check_world_baselink_frames_) CheckOdometryFrameIDs(message);

  // stamp transforms using world and baselink frame IDs from PoseLookup
  geometry_msgs::TransformStamped tf_stamped;
  tf_stamped.header = message->header;
  tf_stamped.header.frame_id = pose_lookup_.GetWorldFrameID();
  tf_stamped.child_frame_id = pose_lookup_.GetBaselinkFrameID();
  tf_stamped.transform.translation.x = message->pose.pose.position.x;
  tf_stamped.transform.translation.y = message->pose.pose.position.y;
  tf_stamped.transform.translation.z = message->pose.pose.position.z;
  tf_stamped.transform.rotation = message->pose.pose.orientation;
  std::string authority{"odometry"};
  poses_->setTransform(tf_stamped, authority, false);
}

}  // namespace frame_initializers
}  // namespace beam_models
