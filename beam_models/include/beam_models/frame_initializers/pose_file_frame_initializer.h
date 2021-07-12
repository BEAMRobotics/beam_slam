#pragma once

#include <beam_models/frame_initializers/frame_initializer_base.h>

namespace beam_models {
namespace frame_initializers {

/**
 * @brief This class can be used to estimate a pose of a frame given its
 * timestamp. This is done by building a tf tree with incoming odometry messages
 * then looking up the transform at the given time.
 *
 */
class PoseFileFrameInitializer : public FrameInitializerBase {
 public:
  /**
   * @brief Constructor
   * @param file_path full path to pose file
   * @param sensor_frame_id frame ID attached to the sensor, used to lookup
   * extrinsic calibrations. See FrameInitializerBase for description
   */
  PoseFileFrameInitializer(const std::string& file_path,
                           const std::string& sensor_frame_id = "");
};

}  // namespace frame_initializers
}  // namespace beam_models