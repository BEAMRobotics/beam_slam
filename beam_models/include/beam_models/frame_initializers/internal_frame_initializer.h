#pragma once

#include <beam_models/frame_initializers/frame_initializer_base.h>
#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <beam_utils/utils.h>

namespace beam_models {
namespace frame_initializers {

/**
 * @brief This class can be used to initialize sensor frames given pose
 * estimates from other sources
 */
class InternalFrameInitializer : public FrameInitializerBase {
 public:
  /**
   * @brief Static Instance getter (singleton)
   * @param poses_buffer_time length of time (in seconds) to store poses for
   * interpolation
   * @return reference to the singleton
   */
  static InternalFrameInitializer& GetInstance(int64_t poses_buffer_time = 30);

  /**
   * @brief Delete copy constructor
   */
  InternalFrameInitializer(InternalFrameInitializer const&) = delete;

  /**
   * @brief Delete assignment constructor
   */
  void operator=(InternalFrameInitializer const&) = delete;

  /**
   * @brief Adds a pose to the tf buffercore
   * @param T_WORLD_SENSOR reference to result
   * @param time stamp of the pose being added
   * @param sensor_frame sensor frame id. If empty, the sensor frame will be
   * baselink. This option is useful when multiple sensors are being used
   * @return true if pose was added successfully
   */
  bool AddPose(const Eigen::Matrix4d& T_WORLD_SENSOR, const ros::Time& stamp,
               std::string sensor_frame_id = "");

 private:
  /**
   * @brief Private Constructor
   * @param poses_buffer_time length of time (in seconds) to store poses for
   * interpolation
   */
  InternalFrameInitializer(int64_t poses_buffer_time);

  int64_t poses_buffer_time_;
};

}  // namespace frame_initializers
}  // namespace beam_models