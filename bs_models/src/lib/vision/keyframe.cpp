#include <bs_models/vision/keyframe.h>

namespace bs_models { namespace vision {

Keyframe::Keyframe(const bs_common::CameraMeasurementMsg& msg) : msg_(msg) {
  static uint64_t kf_seq_num_ = 0;
  sequence_number_ = kf_seq_num_++;
}

const ros::Time& Keyframe::Stamp() const {
  return msg_.header.stamp;
}

const sensor_msgs::Image& Keyframe::Image() const {
  return msg_.image;
}

const uint64_t& Keyframe::SequenceNumber() const {
  return sequence_number_;
}

void Keyframe::AddPose(const ros::Time& timestamp,
                       const Eigen::Matrix4d& T_keyframe_frame) {
  trajectory_[timestamp] = T_keyframe_frame;
}

const bs_common::CameraMeasurementMsg& Keyframe::MeasurementMessage() const {
  return msg_;
}

const std::map<ros::Time, Eigen::Matrix4d>& Keyframe::Trajectory() const {
  return trajectory_;
}

}} // namespace bs_models::vision