#pragma once

#include <beam_utils/utils.h>
#include <bs_common/CameraMeasurementMsg.h>
#include <sensor_msgs/Image.h>

namespace bs_models { namespace vision {

class Keyframe {
public:
  /**
   * @brief Custom constructor
   */
  Keyframe(const bs_common::CameraMeasurementMsg& msg);

  /**
   * @brief Read only access to the timestamp
   */
  const ros::Time& Stamp() const;

  /**
   * @brief Read only access to image message
   */
  const sensor_msgs::Image& Image() const;

  /**
   * @brief Read only access to sequence number
   */
  const uint64_t& SequenceNumber() const;

  /**
   * @brief Read only access to measurement message
   */
  const bs_common::CameraMeasurementMsg& MeasurementMessage() const;

  /**
   * @brief Adds a relative pose to this keyframes trajectory
   * @param timestamp of pose to add
   * @param T_frame_keyframe relative pose from current keyframe to this frame
   */
  void AddPose(const ros::Time& timestamp,
               const Eigen::Matrix4d& T_frame_keyframe);

  /**
   * @brief Read only access to this keyframes trajectory
   * <time, T_frame_keyframe>
   */
  const std::map<uint64_t, Eigen::Matrix4d>& Trajectory() const;

protected:
  ros::Time timestamp_;
  bs_common::CameraMeasurementMsg msg_;
  uint64_t sequence_number_;
  std::map<uint64_t, Eigen::Matrix4d> trajectory_;
};

}} // namespace bs_models::vision
