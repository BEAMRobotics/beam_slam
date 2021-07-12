#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf2/buffer_core.h>

#include <beam_common/pose_lookup.h>

namespace beam_models {
namespace frame_initializers {

/**
 * @brief This base class shows the contract between a FrameInitializer class.
 * The goal of this class is to initialize the pose of a frame given some
 * timestamp. This can simply be from a published topic, or can use an odometry
 * methodology with input sensor data. For more information on the frames, see
 * the PoseLookup class.
 *
 * All input data to the derived classes should be added in a custom constructor
 *
 */
class FrameInitializerBase {
 public:
  /**
   * @brief Constructor
   * @param sensor_frame_id frame ID attached to the sensor, used to lookup
   * extrinsic calibrations. If not supplied, the sensor frame is assumed to
   * coincide with the baselink frame specified in PoseLookup
   */
  FrameInitializerBase(const std::string& sensor_frame_id)
      : sensor_frame_id_{sensor_frame_id} {
    if (sensor_frame_id_.empty())
      sensor_frame_id_ = pose_lookup_.GetBaselinkFrameID();
  };

  /**
   * @brief Gets estimated pose of sensor frame wrt world frame
   * @param T_WORLD_SENSOR reference to result
   * @param time stamp of the frame being initialized
   * @return true if pose lookup was successful
   */
  bool GetEstimatedPose(Eigen::Matrix4d& T_WORLD_SENSOR,
                        const ros::Time& time) {
    return pose_lookup_.GetT_WORLD_SENSOR(T_WORLD_SENSOR, sensor_frame_id_,
                                          time);
  };

 protected:
  beam_common::PoseLookup pose_lookup_;
  std::shared_ptr<tf2::BufferCore> poses_{nullptr};
  std::string sensor_frame_id_{""};
};

}  // namespace frame_initializers
}  // namespace beam_models