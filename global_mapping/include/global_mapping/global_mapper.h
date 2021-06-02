#pragma once

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/uuid.h>
#include <fuse_models/common/throttled_callback.h>

#include <global_mapping/SlamChunkMsg.h>
#include <global_mapping/global_map.h>
#include <beam_parameters/models/global_mapper_params.h>

namespace global_mapping {

/**
 * @brief This is class is implemented as a sensor model which takes in
 * SlamChunk messages, creates and populates a GlobalMap
 */
class GlobalMapper : public fuse_core::AsyncSensorModel {
 public:
  SMART_PTR_DEFINITIONS(GlobalMapper);

  /**
   * @brief This initializes the AsyncSensorModel class, device id and ROS
   * message callback
   */
  GlobalMapper();

  /**
   * @brief default  destructor
   */
  ~GlobalMapper() override = default;

  /**
   * @brief This function takes a slam chunk ROS message and adds it to the
   * global map which will then decide whether to add to the current submap, or
   * generate another. When a new submap is generated two things happen:
   *
   * (1) a new transaction is generated which adds a constraint between the new
   * submap and the previous submap.
   *
   * (2) a loop closure check is initiated with the previous submap and if a
   * loop is found, a constraint is added by creating another transaction to
   * send to the graph.
   *
   * @param msg slam chunk message which may contain lidar data, camera data,
   * and/or pose data
   */
  void process(const SlamChunkMsg::ConstPtr& msg);

 private:
  /**
   * @brief Load and validate params, initialize required variables/pointers
   */
  void onInit() override;

  /**
   * @brief Save final trajectories and maps if specified
   */
  void onStop() override;

  /**
   * @brief Update submap poses
   */
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  fuse_core::UUID device_id_;  //!< The UUID of this device
  beam_parameters::models::GlobalMapperParams params_;
  std::unique_ptr<GlobalMap> global_map_;

  using ThrottledCallback =
      fuse_models::common::ThrottledCallback<SlamChunkMsg>;
  ThrottledCallback throttled_callback_;
};

}  // namespace global_mapping
