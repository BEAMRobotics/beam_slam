#pragma once

#include <beam_models/frame_initializers/frame_initializer_base.h>
#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <beam_utils/utils.h>

namespace beam_models { namespace frame_initializers {

/**
 * @brief This class can be used to estimate a pose of a frame given its
 * timestamp. This is done by building searching through the current fuse graph,
 * or preintegrating to the time point
 */
class InternalFrameInitializer : public FrameInitializerBase {
public:
  /**
   * @brief Static Instance getter (singleton)
   * @return reference to the singleton
   */
  static InternalFrameInitializer& GetInstance();

  /**
   * @brief Gets estimate frame pose (in world frame in imu coords)
   * @param time stamp of the frame being initialized
   * @param T_WORLD_SENSOR reference to result
   * @param frame_id id of the desired output frame
   * @return true if pose lookup was successful
   */
  bool GetEstimatedPose(const ros::Time& time, Eigen::Matrix4d& T_WORLD_SENSOR,
                        std::string frame_id = "") override;

  /**
   * @brief Helper function to add an orientation variable
   * @param stamp timestamp of orientation to add
   * @param R_WORLD_SENSOR orientation variable to add
   * @param frame_id id of the desired input frame
   */
  void AddPose(const ros::Time& stamp, const Eigen::Matrix4d& T_WORLD_SENSOR,
               std::string frame_id);

  /**
   * @brief Delete copy constructor
   */
  InternalFrameInitializer(InternalFrameInitializer const&) = delete;

  /**
   * @brief Delete assignment constructor
   */
  void operator=(InternalFrameInitializer const&) = delete;

private:
  /**
   * @brief Private Default Constructor
   */
  InternalFrameInitializer() {}

  beam_common::PoseLookup::Params pose_lookup_params_;
  std::unique_ptr<beam_common::PoseLookup> pose_lookup_;
  std::shared_ptr<tf2::BufferCore> poses_;
};

}} // namespace beam_models::frame_initializers