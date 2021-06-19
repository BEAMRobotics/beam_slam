#pragma once

#include <tf2/buffer_core.h>

#include <beam_common/pose_lookup.h>
#include <beam_models/frame_initializers/frame_initializer_base.h>

namespace beam_models {
namespace frame_initializers {

/**
 * @brief This class can be used to estimate a pose of a frame given its
 * timestamp. This is done by building a tf tree with incoming odometry messages
 * then looking up the transform at the given time. For more information on the
 * frames, see the PoseLookup class.
 *
 */
class PoseFileFrameInitializer : public FrameInitializerBase {
 public:
  /**
   * @brief Constructor
   * @param file_path full path to pose file
   * @param sensor_frame_id frame ID attached to the sensor, used to lookup
   * extrinsic calibrations. If not supplied, it will assume the odometry is
   * in the baslink frame
   */
  PoseFileFrameInitializer(const std::string& file_path,
                           const std::string& sensor_frame_id = "");

  /**
   * @brief Gets estimate frame pose
   * @param time stamp of the frame being initialized
   * @param T_WORLD_SENSOR reference to result
   * @return true if pose lookup was successful
   */
  bool GetEstimatedPose(const ros::Time& time,
                        Eigen::Matrix4d& T_WORLD_SENSOR) override;

 private:
  beam_common::PoseLookup& pose_lookup_ =
      beam_common::PoseLookup::GetInstance();
  std::string sensor_frame_id_;
};

}  // namespace frame_initializers
}  // namespace beam_models