#pragma once

#include <beam_models/frame_initializers/frame_initializer_base.h>
#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <beam_utils/utils.h>

namespace beam_models { namespace frame_initializers {

/**
 * @brief This class can be used to estimate a pose of a frame given its
 * timestamp. This is done by building a tf tree with incoming odometry messages
 * then looking up the transform at the given time. For more information on the
 * frames, see the PoseLookup class.
 *
 */
class GraphFrameInitializer : public FrameInitializerBase {
public:
  /**
   * @brief Default Constructor
   */
  GraphFrameInitializer();

  /**
   * @brief Gets estimate frame pose (in world frame in imu coords)
   * @param time stamp of the frame being initialized
   * @param T_WORLD_SENSOR reference to result
   * @return true if pose lookup was successful
   */
  bool GetEstimatedPose(const ros::Time& time,
                        Eigen::Matrix4d& T_WORLD_SENSOR) override;

  /**
   * @brief Helper function to get an orientation variable
   * @param track feature track of current image
   */
  fuse_variables::Orientation3DStamped::SharedPtr
      GetOrientation(const ros::Time& stamp);

  /**
   * @brief Helper function to get a position variable
   * @param track feature track of current image
   */
  fuse_variables::Position3DStamped::SharedPtr
      GetPosition(const ros::Time& stamp);

  /**
   * @brief Helper function to get a position variable
   * @param track feature track of current image
   */
  void AddOrientation(
      const ros::Time& stamp,
      const fuse_variables::Orientation3DStamped::SharedPtr& R_WORLD_SENSOR);

  /**
   * @brief Helper function to get a position variable
   * @param track feature track of current image
   */
  void AddPosition(
      const ros::Time& stamp,
      const fuse_variables::Position3DStamped::SharedPtr& t_WORLD_SENSOR);

  /**
   * @brief Gets estimate frame pose
   * @param time stamp of the frame being initialized
   * @param T_WORLD_SENSOR reference to result
   * @return true if pose lookup was successful
   */
  void SetGraph(const fuse_core::Graph::ConstSharedPtr& graph);

  /**
   * @brief Gets estimate frame pose
   * @param preint stamp of the frame being initialized
   * @return true if pose lookup was successful
   */
  void SetIMUPreintegrator(
      std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration>
          imu_preint);

private:
  std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration> imu_preint_;
  fuse_core::Graph::ConstSharedPtr graph_;
  std::unordered_map<uint64_t, fuse_variables::Orientation3DStamped::SharedPtr>
      orientations_;
  std::unordered_map<uint64_t, fuse_variables::Position3DStamped::SharedPtr>
      positions_;
};

}} // namespace beam_models::frame_initializers