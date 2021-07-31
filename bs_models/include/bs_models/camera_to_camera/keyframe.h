#pragma once

// libbeam
#include <beam_utils/utils.h>
#include <sensor_msgs/Image.h>

namespace bs_models { namespace camera_to_camera {

class Keyframe {
public:
  /**
   * @brief Custom cosntrcutor
   * @param cam_model camera model being used
   */
  Keyframe(const ros::Time& timestamp, const sensor_msgs::Image& image);

  /**
   * @brief Read only access to the timestamp
   */
  const ros::Time& Stamp();

  /**
   * @brief Read only access to image message
   */
  const sensor_msgs::Image& Image();

  /**
   * @brief Read only access to sequence number
   */
  const uint64_t& SequenceNumber();

  /**
   * @brief Adds a relative pose to this keyframes trajectory
   * @param timestamp of pose to add
   * @param T_frame_keyframe relative pose from current keyframe to this frame
   */
  void AddPose(const ros::Time& timestamp,
               const Eigen::Matrix4d& T_frame_keyframe);

  /**
   * @brief Read only access to this keyframes trajectory
   */
  const std::map<uint64_t, Eigen::Matrix4d>& Trajectory();

protected:
  ros::Time timestamp_;
  sensor_msgs::Image image_;
  uint64_t sequence_numer_;
  std::map<uint64_t, Eigen::Matrix4d> trajectory_;
};

}} // namespace bs_models::camera_to_camera
