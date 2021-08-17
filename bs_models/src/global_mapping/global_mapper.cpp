#include <bs_models/global_mapping/global_mapper.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>
#include <boost/filesystem.hpp>

#include <beam_utils/math.h>
#include <beam_utils/time.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::global_mapping::GlobalMapper,
                       fuse_core::SensorModel)

namespace bs_models {

namespace global_mapping {

GlobalMapper::GlobalMapper()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_callback_(
          std::bind(&GlobalMapper::process, this, std::placeholders::_1)) {}

void GlobalMapper::process(const SlamChunkMsg::ConstPtr& msg) {
  ros::Time stamp = msg->stamp;
  std::vector<float> T = msg->T_WORLD_BASELINK;
  Eigen::Matrix4d T_WORLD_BASELINK = beam::VectorToEigenTransform(T);

  fuse_core::Transaction::SharedPtr new_transaction =
      global_map_->AddMeasurement(
          msg->camera_measurement, msg->lidar_measurement,
          msg->trajectory_measurement, T_WORLD_BASELINK, stamp);

  if (new_transaction != nullptr) {
    sendTransaction(new_transaction);
  }
}

void GlobalMapper::onInit() {
  params_.loadFromROS(private_node_handle_);
  global_params_.loadFromROS(private_node_handle_);
}

void GlobalMapper::onStart() {
  subscriber_ = node_handle_.subscribe(params_.input_topic, 100,
                                       &ThrottledCallback::callback,
                                       &throttled_callback_);
  // get intrinsics
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(global_params_.cam_intrinsics_path);

  // get extrinsics
  bs_common::ExtrinsicsLookupOnline& extrinsics_online =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
  std::shared_ptr<bs_common::ExtrinsicsLookupBase> extrinsics =
      std::make_shared<bs_common::ExtrinsicsLookupBase>(
          extrinsics_online.GetExtrinsicsCopy());

  // init global map
  if (!params_.global_map_config.empty()) {
    global_map_ = std::make_unique<GlobalMap>(camera_model, extrinsics,
                                              params_.global_map_config);
  } else {
    global_map_ = std::make_unique<GlobalMap>(camera_model, extrinsics);
  }
};

void GlobalMapper::onStop() {
  if (!boost::filesystem::exists(params_.output_path)) {
    BEAM_ERROR("Output path does not exist, not saing results.");
    return;
  }

  std::string dateandtime =
      beam::ConvertTimeToDate(std::chrono::system_clock::now());
  std::string save_path =
      params_.output_path + dateandtime + "_global_mapper_results/";
  boost::filesystem::create_directory(save_path);

  global_map_->SaveTrajectoryFile(save_path,
                                  params_.save_local_mapper_trajectory);

  if (params_.save_global_map_data) {
    std::string global_map_path = save_path + "/GlobalMapData/";
    boost::filesystem::create_directory(global_map_path);
    global_map_->SaveData(global_map_path);
  }

  if (params_.save_trajectory_cloud) {
    global_map_->SaveTrajectoryClouds(save_path,
                                      params_.save_local_mapper_trajectory);
  }
  if (params_.save_submap_frames) {
    global_map_->SaveSubmapFrames(save_path,
                                  params_.save_local_mapper_trajectory);
  }
  if (params_.save_submaps) {
    global_map_->SaveLidarSubmaps(save_path, params_.save_local_mapper_maps);
    global_map_->SaveKeypointSubmaps(save_path, params_.save_local_mapper_maps);
  }
  subscriber_.shutdown();
}

void GlobalMapper::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  global_map_->UpdateSubmapPoses(graph_msg);
}

}  // namespace global_mapping

}  // namespace bs_models
