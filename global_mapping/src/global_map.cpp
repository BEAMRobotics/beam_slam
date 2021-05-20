#include <global_mapping/global_map.h>
// #include <global_mapping/loop_closures.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>

#include <beam_utils/log.h>

namespace global_mapping {

void GlobalMap::Params::LoadJson(const std::string& config_path) {
  nlohmann::json J;
  std::ifstream file(config_path);
  file >> J;
  submap_size = J["submap_size_m"];
  loop_closure_type = J["loop_closure_type"];
  loop_closure_config = J["loop_closure_config"];
}

GlobalMap::GlobalMap(const std::string& config_path) {
  if (!boost::filesystem::exists(config_path)) {
    BEAM_ERROR(
        "GlobalMap config file not found, using default parameters. Input: {}",
        config_path);
    return;
  }

  if (!config_path.empty()) {
    params_.LoadJson(config_path);
  }
  InitiateLoopClosure();
}

void GlobalMap::InitiateLoopClosure() {
  std::string& type = params_.loop_closure_type;

  // if (type == "EUCDISTICP") {
  //   loop_closure_ =
  //       std::make_unique<EucDistIcpLoopClosure>(params_.loop_closure_config);
  // } else if (type == "EUCDISTGICP") {
  //   loop_closure_ =
  //       std::make_unique<EucDistGicpLoopClosure>(params_.loop_closure_config);
  // } else if (type == "EUCDISTNDT") {
  //   loop_closure_ =
  //       std::make_unique<EucDistNdtLoopClosure>(params_.loop_closure_config);
  // } else if (type == "EUCDISTLOAM") {
  //   loop_closure_ =
  //       std::make_unique<EucDistLoamLoopClosure>(params_.loop_closure_config);
  // } else {
  //   BEAM_ERROR("Invalid loop closure type. Using default: EUCDISTICP");
  //   loop_closure_ =
  //       std::make_unique<EucDistIcpLoopClosure>(params_.loop_closure_config);
  // }
}

void GlobalMap::AddCameraMeasurement(const CameraMeasurement& measurement) {
  if (measurement.size == 0) {
    return;
  }
  // float T[measurement.size] = {0};
  std::vector<float> T;
  T = measurement.T_WORLD_FRAME;
  Eigen::Matrix4d T_WORLD_FRAME =
      VectorToEigenTransform(measurement.T_WORLD_FRAME);
  int submap_id = GetSubmapId(T_WORLD_FRAME);

  // if id is equal to submap size then we need to create a new submap
  if (submap_id == submaps_.size()) {
    submaps_.push_back(Submap(measurement.stamp, T_WORLD_FRAME));
  }

  std::vector<LandmarkMeasurement> landmarks;
  for (int i = 0; i < measurement.size; i++) {
    landmarks.push_back(measurement.landmarks[i]);
  }

  submaps_[submap_id].AddCameraMeasurement(
      landmarks, T_WORLD_FRAME, measurement.stamp, measurement.sensor_id,
      measurement.measurement_id);
}

void GlobalMap::AddLidarMeasurement(const LidarMeasurement& measurement) {
  if (measurement.size == 0) {
    return;
  }
  Eigen::Matrix4d T_WORLD_FRAME =
      VectorToEigenTransform(measurement.T_WORLD_FRAME);
  int submap_id = GetSubmapId(T_WORLD_FRAME);

  // if id is equal to submap size then we need to create a new submap
  if (submap_id == submaps_.size()) {
    submaps_.push_back(Submap(measurement.stamp, T_WORLD_FRAME));
  }

  PointCloud cloud;
  submaps_[submap_id].AddLidarMeasurement(
      cloud, T_WORLD_FRAME, measurement.stamp, measurement.sensor_id,
      measurement.measurement_id, measurement.type);
}

void GlobalMap::AddTrajectoryMeasurement(
    const TrajectoryMeasurement& measurement) {
  if (measurement.size == 0) {
    return;
  }
  Eigen::Matrix4d T_WORLD_FRAME =
      VectorToEigenTransform(measurement.T_WORLD_FRAME);
  int submap_id = GetSubmapId(T_WORLD_FRAME);

  // if id is equal to submap size then we need to create a new submap
  if (submap_id == submaps_.size()) {
    submaps_.push_back(Submap(measurement.stamp, T_WORLD_FRAME));
  }

  std::vector<Eigen::Matrix4d, pose_allocator> poses;
  std::vector<ros::Time> stamps;
  for (int i = 0; i < measurement.size; i++) {
    std::vector<float> current_pose;
    for (int j = 0; j < 12; j++) {
      current_pose.push_back(measurement.poses[12 * i + j]);
    }
    poses.push_back(VectorToEigenTransform(current_pose));
    stamps.push_back(ros::Time(measurement.stamps[i]));
  }

  // TODO: make sure ALL Add***Measurement() functions convert from the local
  // mapper world frame to the global mapper world frame
  submaps_[submap_id].AddTrajectoryMeasurement(
      poses, stamps, T_WORLD_FRAME, measurement.stamp, measurement.sensor_id,
      measurement.measurement_id);
}

int GlobalMap::GetSubmapId(const Eigen::Matrix4d& T_WORLD_FRAME) {
  // check if current pose is within "submap_size" from previous submap, or
  // current submap. We prioritize the previous submap for the case where data
  // isn't coming in in order (e.g., lidar data may come in slower)
  Eigen::Vector3d t_WORLD_FRAME = T_WORLD_FRAME.block(0, 3, 3, 3);
  Eigen::Vector3d t_WORLD_SUBMAPPREV =
      submaps_[submaps_.size() - 2].T_WORLD_SUBMAP_INIT().block(0, 3, 3, 3);
  Eigen::Vector3d t_WORLD_SUBMAPCUR =
      submaps_[submaps_.size() - 1].T_WORLD_SUBMAP_INIT().block(0, 3, 3, 3);

  if ((t_WORLD_FRAME - t_WORLD_SUBMAPPREV).norm() < params_.submap_size) {
    return submaps_.size() - 2;
  } else if ((t_WORLD_FRAME - t_WORLD_SUBMAPCUR).norm() < params_.submap_size) {
    return submaps_.size() - 1;
  } else {
    return submaps_.size();
  }
}

fuse_core::Transaction::SharedPtr GlobalMap::FindLoopClosures() {
  // int current_index = submaps_.size() - 1;
  // std::vector<int> matched_indices;
  // if (!loop_closure_->CandidateSearch(submaps_, current_index,
  //                                     matched_indices)) {
  //   return;
  // }

  fuse_core::Transaction::SharedPtr transaction =
      std::make_shared<fuse_core::Transaction>();
  // for (int matched_index : matched_indices) {
  //   auto new_transaction = loop_closure_->GetConstraint(
  //       submaps_[matched_index], submaps_[current_index]);
  //   transaction->merge(new_transaction);
  // }
  return transaction;
}

void GlobalMap::UpdateSubmapPoses(fuse_core::Graph::ConstSharedPtr graph_msg) {
  for (auto& submap : submaps_) {
    submap.UpdatePose(graph_msg);
  }
}

void GlobalMap::SaveLidarSubmaps(const std::string& output_path) {
  if (!boost::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path, not saving submaps. Input: {}",
               output_path);
  }
  std::string submaps_path = output_path + "lidar_submaps/";
  boost::filesystem::create_directory(submaps_path);
  for (int i = 0; i < submaps_.size(); i++) {
    std::string submap_name =
        submaps_path + "lidar_submap" + std::to_string(i) + ".pcd";
    submaps_[i].SaveLidarMapInWorldFrame(submap_name);
  }
}

void GlobalMap::SaveKeypointSubmaps(const std::string& output_path) {
  if (!boost::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path, not saving submaps. Input: {}",
               output_path);
  }
  std::string submaps_path = output_path + "keypoint_submaps/";
  boost::filesystem::create_directory(submaps_path);
  for (int i = 0; i < submaps_.size(); i++) {
    std::string submap_name =
        submaps_path + "keypoint_submap" + std::to_string(i) + ".pcd";
    submaps_[i].SaveKeypointsMapInWorldFrame(submap_name);
  }
}

void GlobalMap::SaveFullLidarMap(const std::string& output_path) {
  if (!boost::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path, not saving map. Input: {}", output_path);
  }
  PointCloud keypoints_map;
  for (int i = 0; i < submaps_.size(); i++) {
    keypoints_map += submaps_[i].GetKeypointsInWorldFrame();
  }
  std::string filename = output_path + "keypoints_map.pcd";
  BEAM_INFO("Saving keypoints map of size {} to: {}", keypoints_map.size(),
            filename);
  pcl::io::savePCDFileBinary(filename, keypoints_map);
}

void GlobalMap::SaveFullKeypointMap(const std::string& output_path) {
  if (!boost::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path, not saving map. Input: {}", output_path);
  }
  PointCloud lidar_map;
  for (int i = 0; i < submaps_.size(); i++) {
    lidar_map += submaps_[i].GetLidarPointsInWorldFrame();
  }
  std::string filename = output_path + "lidar_map.pcd";
  BEAM_INFO("Saving lidar map of size {} to: {}", lidar_map.size(), filename);
  pcl::io::savePCDFileBinary(filename, lidar_map);
}

Eigen::Matrix4d GlobalMap::VectorToEigenTransform(const std::vector<float>& v) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 3) = v[3];
  T(1, 3) = v[7];
  T(2, 3) = v[11];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      int vector_index = i + j * 4;
      T(i, j) = static_cast<double>(v[vector_index]);
    }
  }
}

void GlobalMap::SaveTrajectoryFile(const std::string& output_path) {
  // TODO
}

void GlobalMap::SaveTrajectoryCloud(const std::string& output_path) {
  // TODO
}

}  // namespace global_mapping
