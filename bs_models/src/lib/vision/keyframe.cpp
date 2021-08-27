#include <bs_models/vision/keyframe.h>

namespace bs_models {
namespace vision {

Keyframe::Keyframe(const ros::Time& timestamp, const sensor_msgs::Image& image)
    : timestamp_(timestamp), image_(image) {
  static uint64_t kf_seq_num_ = 0;
  sequence_numer_ = kf_seq_num_++;
}

const ros::Time& Keyframe::Stamp() { return timestamp_; }

const sensor_msgs::Image& Keyframe::Image() { return image_; }

const uint64_t& Keyframe::SequenceNumber() { return sequence_numer_; }

void Keyframe::AddPose(const ros::Time& timestamp,
                       const Eigen::Matrix4d& T_frame_keyframe) {
  trajectory_[timestamp.toNSec()] = T_frame_keyframe;
}

const std::map<uint64_t, Eigen::Matrix4d>& Keyframe::Trajectory() {
  return trajectory_;
}

}  // namespace vision
}  // namespace bs_models