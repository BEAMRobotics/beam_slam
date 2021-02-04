#include <beam_publishers/path_3d_publisher.h>
#include <fuse_core/async_publisher.h>
#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

// Register this publisher with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(beam_publishers::Path3DPublisher, fuse_core::Publisher);

namespace beam_publishers {

Path3DPublisher::Path3DPublisher()
    : fuse_core::AsyncPublisher(1),
      device_id_(fuse_core::uuid::NIL),
      frame_id_("map") {}

void Path3DPublisher::onInit() {
  // Configure the publisher
  std::string device_str;
  if (private_node_handle_.getParam("device_id", device_str)) {
    device_id_ = fuse_core::uuid::from_string(device_str);
  } else if (private_node_handle_.getParam("device_name", device_str)) {
    device_id_ = fuse_core::uuid::generate(device_str);
  }
  if (!private_node_handle_.getParam("frame_id", frame_id_)) {
    frame_id_ = "odom";
  }
  std::string path_topic;
  if (!private_node_handle_.getParam("path_topic", path_topic)) {
    path_topic = "path";
  }
  std::string pose_array_topic;
  if (!private_node_handle_.getParam("pose_array_topic", pose_array_topic)) {
    pose_array_topic = "pose_array";
  }

  // Advertise the topic
  path_publisher_ =
      private_node_handle_.advertise<nav_msgs::Path>(path_topic, 1);
  pose_array_publisher_ =
      private_node_handle_.advertise<geometry_msgs::PoseArray>(pose_array_topic,
                                                               1);
}

void Path3DPublisher::notifyCallback(
    fuse_core::Transaction::ConstSharedPtr /*transaction*/,
    fuse_core::Graph::ConstSharedPtr graph) {
  // Exit early if no one is listening
  if ((path_publisher_.getNumSubscribers() == 0) &&
      (pose_array_publisher_.getNumSubscribers() == 0)) {
    return;
  }
  
  // Extract all of the 3D pose variables to the path
  std::vector<geometry_msgs::PoseStamped> poses;
  for (const auto& variable : graph->getVariables()) {
    auto orientation =
        dynamic_cast<const fuse_variables::Orientation3DStamped*>(&variable);
    if (orientation && (orientation->deviceId() == device_id_)) {
      const auto& stamp = orientation->stamp();
      auto position_uuid =
          fuse_variables::Position3DStamped(stamp, device_id_).uuid();
      if (!graph->variableExists(position_uuid)) { continue; }
      auto position = dynamic_cast<const fuse_variables::Position3DStamped*>(
          &graph->getVariable(position_uuid));
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = frame_id_;
      pose.pose.position.x = position->x();
      pose.pose.position.y = position->y();
      pose.pose.position.z = position->z();
      pose.pose.orientation =
          tf2::toMsg(tf2::Quaternion(orientation->x(), orientation->y(),
                                     orientation->z(), orientation->w()));
      poses.push_back(std::move(pose));
    }
  }
  
  // Exit if there are no poses
  if (poses.empty()) { return; }
  
  // Sort the poses by timestamp
  auto compare_stamps = [](const geometry_msgs::PoseStamped& pose1,
                           const geometry_msgs::PoseStamped& pose2) {
    return pose1.header.stamp < pose2.header.stamp;
  };
  std::sort(poses.begin(), poses.end(), compare_stamps);
  
  // Define the header for the aggregate message
  std_msgs::Header header;
  header.stamp = poses.back().header.stamp;
  header.frame_id = frame_id_;
  
  // Convert the sorted poses into a Path msg
  if (path_publisher_.getNumSubscribers() > 0) {
    nav_msgs::Path path_msg;
    path_msg.header = header;
    path_msg.poses = poses;
    path_publisher_.publish(path_msg);
  }
  
  // Convert the sorted poses into a PoseArray msg
  if (pose_array_publisher_.getNumSubscribers() > 0) {
    geometry_msgs::PoseArray pose_array_msg;
    pose_array_msg.header = header;
    std::transform(poses.begin(), poses.end(),
                   std::back_inserter(pose_array_msg.poses),
                   [](const geometry_msgs::PoseStamped& pose) {
                     return pose.pose;
                   }); // NOLINT(whitespace/braces)
    pose_array_publisher_.publish(pose_array_msg);
  }
}

} // namespace beam_publishers
