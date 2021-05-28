#include <global_mapping/global_mapper.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(global_mapping::GlobalMapper, fuse_core::SensorModel)

namespace global_mapping {

GlobalMapper::GlobalMapper()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_callback_(
          std::bind(&GlobalMapper::process, this, std::placeholders::_1)) {}

void GlobalMapper::process(const SlamChunkMsg::ConstPtr& msg) {
  fuse_core::Transaction::SharedPtr new_transaction1 =
      global_map_->AddCameraMeasurement(msg->camera_measurement);
  if (new_transaction1 != nullptr) {
    sendTransaction(new_transaction1);
  }
  fuse_core::Transaction::SharedPtr new_transaction2 =
      global_map_->AddLidarMeasurement(msg->lidar_measurement);
  if (new_transaction2 != nullptr) {
    sendTransaction(new_transaction2);
  }
  fuse_core::Transaction::SharedPtr new_transaction3 =
      global_map_->AddTrajectoryMeasurement(msg->trajectory_measurement);
  if (new_transaction3 != nullptr) {
    sendTransaction(new_transaction3);
  }
}

void GlobalMapper::onInit() {
  params_.loadFromROS(private_node_handle_);
  ExtrinsicsLookup::Params extrinsics_params{
      .imu_frame = params_.imu_frame,
      .camera_frame = params_.camera_frame,
      .lidar_frame = params_.lidar_frame,
      .static_extrinsics = params_.static_extrinsics};
  std::shared_ptr<ExtrinsicsLookup> extrinsics =
      std::make_shared<ExtrinsicsLookup>(extrinsics_params);

  if (!params_.global_mapper_config.empty()) {
    global_map_ =
        std::make_unique<GlobalMap>(extrinsics, params_.global_mapper_config);
  } else {
    global_map_ = std::make_unique<GlobalMap>(extrinsics);
  }
}

void GlobalMapper::onStop() {
  global_map_->SaveTrajectoryFiles(params_.output_path);
  if (params_.save_trajectory_cloud) {
    global_map_->SaveTrajectoryClouds(params_.output_path);
  }
  if (params_.save_submaps) {
    global_map_->SaveLidarSubmaps(params_.output_path);
    global_map_->SaveKeypointSubmaps(params_.output_path);
  }
  if (params_.save_final_map) {
    global_map_->SaveFullLidarMap(params_.output_path);
    global_map_->SaveFullKeypointMap(params_.output_path);
  }
}

void GlobalMapper::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  global_map_->UpdateSubmapPoses(graph_msg);
}

}  // namespace global_mapping
