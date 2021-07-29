#include <bs_models/camera_to_camera/keyframe.h>

namespace bs_models { namespace camera_to_camera {

Keyframe::Keyframe(const ros::Time& timestamp, const sensor_msgs::Image& image)
    : timestamp_(timestamp), image_(image) {
  static uint64_t kf_seq_num_ = 0;
  sequence_numer_ = kf_seq_num_++;
}

const ros::Time& Keyframe::Stamp() {
  return timestamp_;
}

const sensor_msgs::Image& Keyframe::Image() {
  return image_;
}

const uint64_t& Keyframe::SequenceNumber() {
  return sequence_numer_;
}

void Keyframe::AddPose(const ros::Time& timestamp,
                       const Eigen::Matrix4d& T_WORLD_BASELINK) {
  trajectory_[timestamp.toNSec()] = T_WORLD_BASELINK;
}

const std::map<uint64_t, Eigen::Matrix4d>& Keyframe::Trajectory() {
  return trajectory_;
}

}} // namespace bs_models::camera_to_camera