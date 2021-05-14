#pragma once

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/uuid.h>
#include <fuse_models/common/throttled_callback.h>

#include <global_mapping/SlamChunk.h>
#include <global_mapping/global_map.h>
#include <beam_parameters/models/global_mapper_params.h>

namespace global_mapping {

/**
 * @brief TODO
 */
class GlobalMapper : public fuse_core::AsyncSensorModel {
 public:
  SMART_PTR_DEFINITIONS(GlobalMapper);

  /**
   * @brief This constructor loads variables, and initializes the
   * frame_initializer object
   */
  GlobalMapper();
      

  ~GlobalMapper() override = default;

  /**
   * @brief TODO
   */
  void process(const SlamChunk::ConstPtr& msg);

 private:
  /**
   * @brief TODO
   */
  void onInit() override;

  /**
   * @brief TODO
   */
  void onStart() override;

  /**
   * @brief TODO
   */
  void onStop() override;

  /**
   * @brief TODO
   */
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  fuse_core::UUID device_id_;  //!< The UUID of this device
  beam_parameters::models::GlobalMapperParams params_;
  GlobalMap global_map_;

  using ThrottledCallback =
      fuse_models::common::ThrottledCallback<SlamChunk>;
  ThrottledCallback throttled_callback_;
};

}  // namespace global_mapping
