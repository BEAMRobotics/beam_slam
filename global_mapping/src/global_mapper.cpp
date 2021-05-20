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

void GlobalMapper::process(const SlamChunk::ConstPtr& msg) {
  global_map_.AddCameraMeasurement(msg->camera_measurement);
  global_map_.AddLidarMeasurement(msg->lidar_measurement);
  global_map_.AddTrajectoryMeasurement(msg->trajectory_measurement);
  fuse_core::Transaction::SharedPtr transaction =
      global_map_.FindLoopClosures();
  ROS_DEBUG("Sending transaction.");
  sendTransaction(transaction);
}

void GlobalMapper::onInit() {
  params_.loadFromROS(private_node_handle_);
  if (!params_.global_mapper_config.empty()) {
    global_map_ = GlobalMap(params_.global_mapper_config);
  }
}

void GlobalMapper::onStart() {
  //
}

void GlobalMapper::onStop() {
  global_map_.SaveTrajectoryFile(params_.output_path);
  if(params_.save_trajectory_cloud){
    global_map_.SaveTrajectoryCloud(params_.output_path);
  }
  if (params_.save_submaps) {
    global_map_.SaveLidarSubmaps(params_.output_path);
    global_map_.SaveKeypointSubmaps(params_.output_path);
  }
  if (params_.save_final_map) {
    global_map_.SaveFullLidarMap(params_.output_path);
    global_map_.SaveFullKeypointMap(params_.output_path);
  }
}

void GlobalMapper::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  global_map_.UpdateSubmapPoses(graph_msg);
}

}  // namespace global_mapping
