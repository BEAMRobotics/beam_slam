#pragma once

#include <beam_models/frame_initializers/frame_initializer_base.h>
#include <beam_common/extrinsics_lookup.h>

namespace beam_models {
namespace frame_initializers {

/**
 * @brief This class can be used to estimate a pose of a frame given its
 * timestamp. This is done by building a tf tree with pose information from a
 * supported pose file. Supported file formats nclude: .json, .txt, .ply
 *
 */
class PoseFileFrameInitializer : public FrameInitializerBase {
 public:
  /**
   * @brief Constructor
   * @param file_path full path to pose file
   */
  PoseFileFrameInitializer(const std::string& file_path);
};

}  // namespace frame_initializers
}  // namespace beam_models