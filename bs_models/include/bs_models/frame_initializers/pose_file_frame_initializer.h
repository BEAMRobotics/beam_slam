#pragma once

#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/frame_initializers/frame_initializer_base.h>

namespace bs_models { namespace frame_initializers {

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

private:
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
};

}} // namespace bs_models::frame_initializers