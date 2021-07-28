#include <global_mapping/global_mapper.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

#include <beam_utils/math.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(global_mapping::GlobalMapper, fuse_core::SensorModel)

namespace global_mapping {

GlobalMapper::GlobalMapper()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_callback_(
          std::bind(&GlobalMapper::process, this, std::placeholders::_1)) {}

void GlobalMapper::process(const SlamChunkMsg::ConstPtr& msg) {
  std::string baselink_frame_id = msg->baselink_frame_id;
  ros::Time stamp = msg->stamp;
  std::vector<float> T = msg->T_WORLD_BASELINK;
  Eigen::Matrix4d T_WORLD_BASELINK = beam::VectorToEigenTransform(T);

  fuse_core::Transaction::SharedPtr new_transaction =
      global_map_->AddMeasurement(msg->camera_measurement,
                                  msg->lidar_measurement,
                                  msg->trajectory_measurement, T_WORLD_BASELINK,
                                  stamp, baselink_frame_id);
  if (new_transaction != nullptr) {
    sendTransaction(new_transaction);
  }
}

void GlobalMapper::onInit() {
  params_.loadFromROS(private_node_handle_);

  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(params_.intrinsics_path);
  if (!params_.global_mapper_config.empty()) {
    global_map_ =
        std::make_unique<GlobalMap>(camera_model, params_.global_mapper_config);
  } else {
    global_map_ = std::make_unique<GlobalMap>(camera_model);
  }
}

void GlobalMapper::onStop() {
  global_map_->SaveTrajectoryFile(params_.output_path,
                                  params_.save_local_mapper_trajectory);
  if (params_.save_trajectory_cloud) {
    global_map_->SaveTrajectoryClouds(params_.output_path,
                                      params_.save_local_mapper_trajectory);
  }
  if (params_.save_submaps) {
    global_map_->SaveLidarSubmaps(params_.output_path,
                                  params_.save_local_mapper_maps);
    global_map_->SaveKeypointSubmaps(params_.output_path,
                                     params_.save_local_mapper_maps);
  }
  if (params_.save_final_map) {
    global_map_->SaveFullLidarMap(params_.output_path,
                                  params_.save_local_mapper_maps);
    global_map_->SaveFullKeypointMap(params_.output_path,
                                     params_.save_local_mapper_maps);
  }
}

void GlobalMapper::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  global_map_->UpdateSubmapPoses(graph_msg);
}

}  // namespace global_mapping
