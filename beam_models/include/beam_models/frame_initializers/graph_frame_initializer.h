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
class GraphFrameInitializer : public FrameInitializerBase {
public:
  /**
   * @brief Static Instance getter (singleton)
   * @return reference to the singleton
   */
  static GraphFrameInitializer& GetInstance();

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
   * @param stamp timestamp of orienation to get
   */
  fuse_variables::Orientation3DStamped::SharedPtr
      GetOrientation(const ros::Time& stamp);

  /**
   * @brief Helper function to get a position variable
   * @param stamp timestamp of position to get
   */
  fuse_variables::Position3DStamped::SharedPtr
      GetPosition(const ros::Time& stamp);

  /**
   * @brief Helper function to add an orientation variable
   * @param stamp timestamp of orientation to add
   * @param R_WORLD_SENSOR orientation variable to add
   */
  void AddOrientation(
      const ros::Time& stamp,
      const fuse_variables::Orientation3DStamped::SharedPtr& R_WORLD_SENSOR);

  /**
   * @brief Helper function to add a position variable
   * @param stamp timestamp of position to add
   * @param t_WORLD_SENSOR position variable to add
   */
  void AddPosition(
      const ros::Time& stamp,
      const fuse_variables::Position3DStamped::SharedPtr& t_WORLD_SENSOR);

  /**
   * @brief Updates the currently held graph and clears local maps
   * @param graph graph to update with
   */
  void SetGraph(const fuse_core::Graph::ConstSharedPtr& graph);

  /**
   * @brief Sets the imu preintegration object
   * @param preint imu preint pointer to set
   */
  void SetIMUPreintegrator(
      std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration>
          imu_preint);

  /**
   * @brief Delete copy constructor
   */
  GraphFrameInitializer(GraphFrameInitializer const&) = delete;

  /**
   * @brief Delete assignment constructor
   */
  void operator=(GraphFrameInitializer const&) = delete;

private:
  /**
   * @brief Private Default Constructor
   */
  GraphFrameInitializer() {}

  std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration> imu_preint_;
  fuse_core::Graph::ConstSharedPtr graph_;
  std::unordered_map<uint64_t, fuse_variables::Orientation3DStamped::SharedPtr>
      orientations_;
  std::unordered_map<uint64_t, fuse_variables::Position3DStamped::SharedPtr>
      positions_;
};

}} // namespace beam_models::frame_initializers