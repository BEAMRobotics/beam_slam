#include <bs_models/global_mapper.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>
#include <boost/filesystem.hpp>

#include <beam_utils/math.h>
#include <beam_utils/log.h>
#include <beam_utils/time.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::GlobalMapper, fuse_core::SensorModel)

namespace bs_models {

GlobalMapper::GlobalMapper()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_callback_(
          std::bind(&GlobalMapper::process, this, std::placeholders::_1)) {}

void GlobalMapper::process(const bs_common::SlamChunkMsg::ConstPtr& msg) {
  ros::Time stamp = msg->stamp;
  std::vector<double> T = msg->T_WORLD_BASELINK;
  Eigen::Matrix4d T_WORLD_BASELINK = beam::VectorToEigenTransform(T);

  std::string matrix_check_summary;
  if (!beam::IsTransformationMatrix(T_WORLD_BASELINK, matrix_check_summary)) {
    BEAM_WARN(
        "transformation matrix invalid, not adding SlamChunkMsg to global map. "
        "Reason: %s, Input:",
        matrix_check_summary.c_str());
    std::cout << "T_WORLD_BASELINK\n" << T_WORLD_BASELINK << "\n";
    return;
  }

  // update extrinsics if necessary
  UpdateExtrinsics();

  // create add to map and transaction
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
    ROS_DEBUG("Sending transaction:");
    graph_->update(*new_transaction);
    ROS_DEBUG("Optimizing graph");
    graph_->optimize();
    ROS_DEBUG("Updating global map");
    global_map_->UpdateSubmapPoses(graph_, ros::Time::now());
    ROS_DEBUG("Global map updated.");
  }

  if (params_.publish_new_submaps || params_.publish_updated_global_map) {
    std::vector<std::shared_ptr<global_mapping::RosMap>> maps_to_publish =
        global_map_->GetRosMaps();
    for (const std::shared_ptr<global_mapping::RosMap>& ros_map :
         maps_to_publish) {
      if (ros_map->first == global_mapping::RosMapType::LIDARSUBMAP) {
        submap_lidar_publisher_.publish(ros_map->second);
      } else if (ros_map->first == global_mapping::RosMapType::VISUALSUBMAP) {
        submap_keypoints_publisher_.publish(ros_map->second);
      } else if (ros_map->first == global_mapping::RosMapType::LIDARGLOBALMAP) {
        global_map_lidar_publisher_.publish(ros_map->second);
      } else if (ros_map->first ==
                 global_mapping::RosMapType::VISUALGLOBALMAP) {
        global_map_keypoints_publisher_.publish(ros_map->second);
      }
    }
  }
}

void GlobalMapper::onInit() {
  params_.loadFromROS(private_node_handle_);
  calibration_params_.loadFromROS();
  graph_ = fuse_graphs::HashGraph::make_shared();
}

void GlobalMapper::onStart() {
  // init subscribers and publishers
  subscriber_ = node_handle_.subscribe<bs_common::SlamChunkMsg>(
      ros::names::resolve(params_.input_topic), 100,
      &ThrottledCallback::callback, &throttled_callback_,
      ros::TransportHints().tcpNoDelay(false));
  if (params_.publish_new_submaps) {
    submap_lidar_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
        params_.new_submaps_topic + "/lidar", 10);
    submap_keypoints_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            params_.new_submaps_topic + "/visual", 10);
  }

  if (params_.publish_updated_global_map) {
    global_map_lidar_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            params_.global_map_topic + "/lidar", 10);
    global_map_keypoints_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            params_.global_map_topic + "/visual", 10);
  }

  // get intrinsics
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(
          calibration_params_.cam_intrinsics_path);

  // get extrinsics
  extrinsics_data_ = std::make_shared<bs_common::ExtrinsicsLookupBase>(
      extrinsics_online_.GetExtrinsicsCopy());

  // init global map
  if (!params_.global_map_config.empty()) {
    global_map_ = std::make_unique<global_mapping::GlobalMap>(
        camera_model, extrinsics_data_, params_.global_map_config);
  } else {
    global_map_ = std::make_unique<global_mapping::GlobalMap>(camera_model,
                                                              extrinsics_data_);
  }
  global_map_->SetStoreNewSubmaps(params_.publish_new_submaps);
  global_map_->SetStoreUpdatedGlobalMap(params_.publish_updated_global_map);
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
    global_map_->SaveKeypointSubmaps(save_path, params_.save_local_mapper_maps);
    global_map_->SaveLidarSubmaps(save_path, params_.save_local_mapper_maps);
  }
  subscriber_.shutdown();
}

void GlobalMapper::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  BEAM_ERROR("UPDATED GRAPH!");
  // uncomment if using sensor model's graph:
  // global_map_->UpdateSubmapPoses(graph_msg, ros::Time::now());
}

void GlobalMapper::UpdateExtrinsics() {
  if (extrinsics_online_.IsStatic() && extrinsics_initialized_) {
    return;
  }

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

}  // namespace bs_models
