#pragma once

#include <tf2/buffer_core.h>

#include <beam_common/pose_lookup.h>
#include <beam_models/frame_initializers/frame_initializer_base.h>

namespace beam_models { namespace frame_initializers {

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
   * @param world_frame_id fixed frame in the poses. If empty, it will use the
   * frame from the pose file.
   * @param baselink_frame_id moving frame in the poses. If empty, it will use
   * the frame from the pose file.
   * @param sensor_frame_id frame ID attached to the sensor, used to lookup
   * extrinsic calibrations. If not supplied, it will assume it is the same
   * frame as the baselink_frame_id
   * @param static_extrinsics set to false if extrinsics change and transforms
   * are broadcasted to /tf
   */
  PoseFileFrameInitializer(const std::string& file_path,
                           const std::string& sensor_frame_id = "",
                           const std::string& baselink_frame_id = "",
                           const std::string& world_frame_id = "",
                           bool static_extrinsics = true);

  /**
   * @brief Gets estimate frame pose
   * @param time stamp of the frame being initialized
   * @param T_WORLD_SENSOR reference to result
   * @return true if pose lookup was successful
   */
  bool GetEstimatedPose(const ros::Time& time, Eigen::Matrix4d& T_WORLD_SENSOR,
                        std::string frame_id = "") override;

private:
  std::unique_ptr<beam_common::PoseLookup> pose_lookup_;
};

}} // namespace beam_models::frame_initializers