#include <bs_models/global_mapper.h>

#include <filesystem>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <beam_utils/time.h>

#include <bs_common/conversions.h>
#include <bs_common/graph_access.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::GlobalMapper, fuse_core::SensorModel)

namespace bs_models {

using namespace global_mapping;

GlobalMapper::GlobalMapper()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_callback_slam_chunk_(std::bind(&GlobalMapper::ProcessSlamChunk,
                                               this, std::placeholders::_1)) {}

void GlobalMapper::ProcessSlamChunk(
    const bs_common::SlamChunkMsg::ConstPtr& msg) {
  ros::Time stamp = msg->T_WORLD_BASELINK.header.stamp;

  Eigen::Matrix4d T_WORLD_BASELINK;
  bs_common::PoseMsgToTransformationMatrix(msg->T_WORLD_BASELINK,
                                           T_WORLD_BASELINK);

  // update extrinsics if necessary
  UpdateExtrinsics();

  // Add to map and create transaction
  fuse_core::Transaction::SharedPtr new_transaction =
      global_map_->AddMeasurement(
          msg->camera_measurement, msg->lidar_measurement,
          msg->trajectory_measurement, T_WORLD_BASELINK, stamp);

  // send transaction if not empty
  if (new_transaction != nullptr) {
    // uncomment if using sensor model's graph:
    // ROS_DEBUG("Sending transaction:");
    // sendTransaction(new_transaction);
    // ROS_DEBUG("Done sending transaction.");

    // uncomment if using self contained graph:
    graph_->update(*new_transaction);
    graph_->optimize();
    global_map_->UpdateSubmapPoses(graph_, ros::Time::now());
  }

  if (params_.publish_new_submaps || params_.publish_updated_global_map ||
      params_.publish_new_scans) {
    std::vector<std::shared_ptr<RosMap>> maps_to_publish =
        global_map_->GetRosMaps();
    for (const std::shared_ptr<RosMap>& ros_map : maps_to_publish) {
      if (ros_map->first == RosMapType::LIDARNEW) {
        new_scans_publisher_.publish(ros_map->second);
      } else if (ros_map->first == RosMapType::LIDARSUBMAP) {
        submap_lidar_publisher_.publish(ros_map->second);
      } else if (ros_map->first == RosMapType::VISUALSUBMAP) {
        submap_keypoints_publisher_.publish(ros_map->second);
      } else if (ros_map->first == RosMapType::LIDARGLOBALMAP) {
        global_map_lidar_publisher_.publish(ros_map->second);
      } else if (ros_map->first == RosMapType::VISUALGLOBALMAP) {
        global_map_keypoints_publisher_.publish(ros_map->second);
      }
    }
  }
}

void GlobalMapper::onInit() {
  // load params
  params_.loadFromROS(private_node_handle_);
  calibration_params_.loadFromROS();

  // init PGO graph
  graph_ = fuse_graphs::HashGraph::make_shared();
}

void GlobalMapper::onStart() {
  // init subscribers and publishers
  slam_chunk_subscriber_ =
      private_node_handle_.subscribe<bs_common::SlamChunkMsg>(
          ros::names::resolve("/local_mapper/slam_results"), 100,
          &ThrottledCallbackSlamChunk::callback,
          &throttled_callback_slam_chunk_,
          ros::TransportHints().tcpNoDelay(false));

  if (params_.publish_new_submaps) {
    submap_lidar_publisher_ =
        private_node_handle_.advertise<sensor_msgs::PointCloud2>(
            "submaps/lidar", 10);
    submap_keypoints_publisher_ =
        private_node_handle_.advertise<sensor_msgs::PointCloud2>(
            "submaps/visual", 10);
  }

  if (params_.publish_new_scans) {
    new_scans_publisher_ =
        private_node_handle_.advertise<sensor_msgs::PointCloud2>("scans", 50);
  }

  if (params_.publish_updated_global_map) {
    global_map_lidar_publisher_ =
        private_node_handle_.advertise<sensor_msgs::PointCloud2>(
            "global_map/lidar", 10);
    global_map_keypoints_publisher_ =
        private_node_handle_.advertise<sensor_msgs::PointCloud2>(
            "global_map/visual", 10);
  }

  // get intrinsics
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(
          calibration_params_.cam_intrinsics_path);

  // get extrinsics
  extrinsics_data_ = std::make_shared<bs_common::ExtrinsicsLookupBase>(
      extrinsics_online_.GetExtrinsicsCopy());

  // init global map
  global_map_ = std::make_unique<GlobalMap>(camera_model, extrinsics_data_,
                                            params_.global_map_config);
  global_map_->SetStoreNewSubmaps(params_.publish_new_submaps);
  global_map_->SetStoreUpdatedGlobalMap(params_.publish_updated_global_map);
  global_map_->SetStoreNewScans(params_.publish_new_scans);

  // setup output
  if (!std::filesystem::exists(params_.output_path)) {
    BEAM_ERROR("Invalid output path: {}", params_.output_path);
    throw std::runtime_error{"invalid output path"};
  } else if (params_.output_path.empty()) {
    BEAM_WARN("No output path provided to global mapper, not saving results");
  } else {
    save_path_ =
        beam::CombinePaths(params_.output_path, "global_mapper_results");
    if (std::filesystem::exists(save_path_)) {
      BEAM_WARN("Clearing existing global mapper results folder");
      std::filesystem::remove_all(save_path_);
    }
    BEAM_INFO("Creating new global mapper results folder: {}", save_path_);
    std::filesystem::create_directory(save_path_);

    if (params_.save_loop_closure_results) {
      std::string reloc_ref_save_path =
          beam::CombinePaths(save_path_, "reloc_refinement_results");
      std::filesystem::create_directory(reloc_ref_save_path);
      global_map_->SetLoopClosureResultsPath(reloc_ref_save_path);
    }
  }
};

void GlobalMapper::onStop() {
  if (trigger_loop_closure_on_stop_) {
    // use beam logging here because ROS logging stops when a node shutdown gets
    // called
    BEAM_INFO("Running final loop closure");
    fuse_core::Transaction::SharedPtr transaction_ptr =
        global_map_->RunLoopClosure();
    if (transaction_ptr) {
      BEAM_INFO("Found {} loop closures. Updating map.",
                bs_common::GetNumberOfConstraints(transaction_ptr));
      graph_->update(*transaction_ptr);
      graph_->optimize();
      global_map_->UpdateSubmapPoses(graph_, ros::Time::now());
    } else {
      BEAM_INFO("No loop closures added on stop.");
    }
  }

  if (!std::filesystem::exists(params_.output_path)) {
    BEAM_ERROR("Output path does not exist, not saving results.");
    return;
  }

  if (!save_path_.empty()) {
    global_map_->SaveTrajectoryFile(save_path_,
                                    params_.save_local_mapper_trajectory);
    if (params_.save_global_map_data) {
      std::string global_map_path =
          beam::CombinePaths(save_path_, "GlobalMapData");
      std::filesystem::create_directory(global_map_path);
      global_map_->SaveData(global_map_path);
    }

    if (params_.save_trajectory_cloud) {
      global_map_->SaveTrajectoryClouds(save_path_,
                                        params_.save_local_mapper_trajectory);
    }

    if (params_.save_submap_frames) {
      global_map_->SaveSubmapFrames(save_path_,
                                    params_.save_local_mapper_trajectory);
    }

    if (params_.save_submaps) {
      global_map_->SaveKeypointSubmaps(save_path_,
                                       params_.save_local_mapper_maps);
      global_map_->SaveLidarSubmaps(save_path_, params_.save_local_mapper_maps);
    }
  }

  // stop subscribers
  slam_chunk_subscriber_.shutdown();
}

void GlobalMapper::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  BEAM_ERROR(
      "UPDATED GRAPH! This shouldn't happen. Make sure you aren't calling the "
      "wrong graph.");
  // uncomment if using sensor model's graph:
  // global_map_->UpdateSubmapPoses(graph_msg, ros::Time::now());
}

void GlobalMapper::UpdateExtrinsics() {
  if (extrinsics_online_.IsStatic() && extrinsics_initialized_) { return; }

  ROS_DEBUG("Updating extrinsics.");
  // update all three transforms
  Eigen::Matrix4d T;
  extrinsics_online_.GetT_LIDAR_IMU(T);
  extrinsics_data_->SetTransform(T, extrinsics_online_.GetLidarFrameId(),
                                 extrinsics_online_.GetImuFrameId());
  extrinsics_online_.GetT_LIDAR_CAMERA(T);
  extrinsics_data_->SetTransform(T, extrinsics_online_.GetLidarFrameId(),
                                 extrinsics_online_.GetCameraFrameId());
  extrinsics_online_.GetT_IMU_CAMERA(T);
  extrinsics_data_->SetTransform(T, extrinsics_online_.GetImuFrameId(),
                                 extrinsics_online_.GetCameraFrameId());
  extrinsics_initialized_ = true;
  ROS_DEBUG("Done updating extrinsics.");
}

} // namespace bs_models
