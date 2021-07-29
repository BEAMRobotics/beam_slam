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

  const ros::Time& Stamp();

  const sensor_msgs::Image& Image();

  const uint64_t& SequenceNumber();

  void AddPose(const ros::Time& timestamp,
               const Eigen::Matrix4d& T_WORLD_BASELINK);

  const std::map<uint64_t, Eigen::Matrix4d>& Trajectory();

protected:
  ros::Time timestamp_;
  sensor_msgs::Image image_;
  uint64_t sequence_numer_;
  std::map<uint64_t, Eigen::Matrix4d> trajectory_;
};

}} // namespace bs_models::camera_to_camera
