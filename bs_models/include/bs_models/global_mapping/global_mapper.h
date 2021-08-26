#pragma once

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/uuid.h>
#include <fuse_models/common/throttled_callback.h>

#include <bs_common/bs_msgs.h>
#include <bs_models/global_mapping/global_map.h>
#include <bs_parameters/models/global_mapper_params.h>
#include <bs_parameters/models/calibration_params.h>

namespace bs_models {

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
   * @brief initi subscriber
   */
  void onStart() override;

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

  /**
   * @brief Update extrinsics if they have not been initialized, or if they are
   * not static. This will use ros::Time(0) which means it takes the most
   * recently available transforms
   */
  void UpdateExtrinsics();

  fuse_core::UUID device_id_;  //!< The UUID of this device
  bs_parameters::models::GlobalMapperParams params_;
  bs_parameters::models::CalibrationParams calibration_params_;

  /** Extrinsics: store shared pointer to base data to pass to global map, then
   * keep a reference to the online intrinsics so that we can update the shared
   * pointer. Also, keep a bool to know if extrinsics have been initialized.
   * NOTE: We have found that if we try to initialize the intrinsics at
   * instantiation of the GlobalMapper, the online extrinsics isn't available
   * yet. So we initialize the first time we receive data.
   */
  std::shared_ptr<bs_common::ExtrinsicsLookupBase> extrinsics_data_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_online_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
  bool extrinsics_initialized_{false};

  std::unique_ptr<GlobalMap> global_map_;

  using ThrottledCallback =
      fuse_models::common::ThrottledCallback<SlamChunkMsg>;
  ThrottledCallback throttled_callback_;

  /** subscribe to slam chunk data */
  ros::Subscriber subscriber_;

  // params that can only be set here:
  int max_output_map_size_{3000000};  // limits output size of lidar maps
};

}  // namespace global_mapping

}  // namespace bs_models
