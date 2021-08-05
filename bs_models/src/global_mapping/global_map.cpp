#include <bs_models/global_mapping/global_map.h>

#include <chrono>
#include <ctime>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>

#include <bs_models/global_mapping/loop_closure/loop_closure_methods.h>
#include <bs_constraints/frame_to_frame/pose_3d_stamped_transaction.h>

#include <beam_utils/log.h>
#include <beam_utils/time.h>
#include <beam_utils/pointclouds.h>
#include <beam_mapping/Poses.h>

namespace bs_models {

namespace global_mapping {

Eigen::Matrix4d VectorToTransform(const std::vector<float>& v) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 0) = v[0];
  T(0, 1) = v[1];
  T(0, 2) = v[2];
  T(0, 3) = v[3];
  T(1, 0) = v[4];
  T(1, 1) = v[5];
  T(1, 2) = v[6];
  T(1, 3) = v[7];
  T(2, 0) = v[8];
  T(2, 1) = v[9];
  T(2, 2) = v[10];
  T(2, 3) = v[11];
  return T;
}

void GlobalMap::Params::LoadJson(const std::string& config_path) {
  BEAM_INFO("Loading global map config file: {}", config_path);

  nlohmann::json J;
  std::ifstream file(config_path);
  file >> J;
  submap_size = J["submap_size_m"];
  loop_closure_candidate_search_type = J["loop_closure_candidate_search_type"];
  loop_closure_refinement_type = J["loop_closure_refinement_type"];
  loop_closure_candidate_search_config =
      J["loop_closure_candidate_search_config"];
  loop_closure_refinement_config = J["loop_closure_refinement_config"];

  std::vector<double> vec;
  Eigen::VectorXd vec_eig(6);
  for (const auto& value : J["local_mapper_covariance_diag"]) {
    vec.push_back(value.get<double>());
  }
  if (vec.size() != 6) {
    BEAM_ERROR(
        "Invalid local mapper covariance diagonal (6 values required). Using "
        "default.");
    vec_eig << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3;
    local_mapper_covariance = vec_eig.asDiagonal();
  }
  vec.clear();

  for (const auto& value : J["loop_closure_covariance_diag"]) {
    vec.push_back(value.get<double>());
  }
  if (vec.size() != 6) {
    BEAM_ERROR(
        "Invalid loop closure covariance diagonal (6 values required). Using "
        "default.");
    vec_eig << 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5;
    loop_closure_covariance = vec_eig.asDiagonal();
  }
}

GlobalMap::GlobalMap(
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model)
    : camera_model_(camera_model) {
  Setup();
}

GlobalMap::GlobalMap(
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
    const Params& params)
    : camera_model_(camera_model), params_(params) {
  Setup();
}

GlobalMap::GlobalMap(
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
    const std::string& config_path)
    : camera_model_(camera_model) {
  std::string read_file = config_path;

  if (config_path == "DEFAULT_PATH") {
    std::string default_path = __FILE__;
    size_t start_iter = default_path.find("bs_models");
    size_t end_iter = default_path.size() - start_iter;
    default_path.erase(start_iter, end_iter);
    default_path += "beam_slam_launch/config/global_map/global_map.json";
    if (!boost::filesystem::exists(default_path)) {
      BEAM_WARN(
          "Could not find default global map config file at: {}. Using "
          "default params.",
          default_path);
      Setup();
      return;
    }
    read_file = default_path;
  } else if (!boost::filesystem::exists(config_path)) {
    BEAM_ERROR(
        "GlobalMap config file not found, using default parameters. Input: {}",
        config_path);
    Setup();
    return;
  }

  if (!config_path.empty()) {
    params_.LoadJson(read_file);
  }
  Setup();
}

void GlobalMap::Setup() {
  // initiate loop closure candidate search
  if (params_.loop_closure_candidate_search_type == "EUCDIST") {
    loop_closure_candidate_search_ =
        std::make_unique<LoopClosureCandidateSearchEucDist>(
            params_.loop_closure_candidate_search_config);
  } else {
    BEAM_ERROR(
        "Invalid loop closure candidate search type. Using default: EUCDIST. "
        "Input: {}",
        params_.loop_closure_candidate_search_type);
    loop_closure_candidate_search_ =
        std::make_unique<LoopClosureCandidateSearchEucDist>(
            params_.loop_closure_candidate_search_config);
  }

  // initiate loop closure refinement
  if (params_.loop_closure_refinement_type == "ICP") {
    loop_closure_refinement_ = std::make_unique<LoopClosureRefinementIcp>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  } else if (params_.loop_closure_refinement_type == "GICP") {
    loop_closure_refinement_ = std::make_unique<LoopClosureRefinementGicp>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  } else if (params_.loop_closure_refinement_type == "NDT") {
    loop_closure_refinement_ = std::make_unique<LoopClosureRefinementNdt>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  } else if (params_.loop_closure_refinement_type == "LOAM") {
    loop_closure_refinement_ = std::make_unique<LoopClosureRefinementLoam>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  } else {
    BEAM_ERROR("Invalid loop closure refinement type. Using default: ICP");
    loop_closure_refinement_ = std::make_unique<LoopClosureRefinementIcp>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  }
}

fuse_core::Transaction::SharedPtr GlobalMap::AddMeasurement(
    const CameraMeasurementMsg& cam_measurement,
    const LidarMeasurementMsg& lid_measurement,
    const TrajectoryMeasurementMsg& traj_measurement,
    const Eigen::Matrix4d& T_WORLD_BASELINK, const ros::Time& stamp) {
  fuse_core::Transaction::SharedPtr new_transaction = nullptr;

  int submap_id = GetSubmapId(T_WORLD_BASELINK);

  // if id is equal to submap size then we need to create a new submap
  if (submap_id == submaps_.size()) {
    submaps_.push_back(Submap(stamp, T_WORLD_BASELINK, camera_model_));

    new_transaction = InitiateNewSubmapPose();

    fuse_core::Transaction::SharedPtr loop_closure_transaction =
        FindLoopClosures();

    if (loop_closure_transaction != nullptr) {
      new_transaction->merge(*loop_closure_transaction);
    }
  }

  // add camera measurement if not empty
  if (!cam_measurement.landmarks.empty()) {
    submaps_[submap_id].AddCameraMeasurement(
        cam_measurement.landmarks, cam_measurement.descriptor_type,
        T_WORLD_BASELINK, stamp, cam_measurement.sensor_id,
        cam_measurement.measurement_id);
  }

  // add lidar measurement if not empty
  if (!lid_measurement.points.empty()) {
    std::vector<float> points = lid_measurement.points;

    // check dimensions of points first
    if (points.size() % 3 != 0) {
      BEAM_ERROR(
          "Invalid size of lidar points. Total number of values is not "
          "divisible by 3. Skipping lidar measurement.");
    } else {
      int num_points = static_cast<int>(points.size() / 3);
      PointCloud cloud;
      for (int iter = 0; iter < num_points; iter += 3) {
        pcl::PointXYZ p;
        p.x = points[iter];
        p.y = points[iter + 1];
        p.z = points[iter + 2];
        cloud.points.push_back(p);
      }
      submaps_[submap_id].AddLidarMeasurement(cloud, T_WORLD_BASELINK, stamp,
                                              lid_measurement.point_type);
    }
  }
  // add trajectory measurement if not empty
  if (!traj_measurement.stamps.empty()) {
    std::vector<float> poses_vec = traj_measurement.poses;
    uint16_t num_poses = static_cast<uint16_t>(poses_vec.size() / 12);

    // check dimensions of inputs first
    if (poses_vec.size() % 12 != 0) {
      BEAM_ERROR(
          "Invalid size of poses vector, number of elements must be divisible "
          "by 4. Not adding trajectory measurement.");
    } else if (num_poses != traj_measurement.stamps.size()) {
      BEAM_ERROR(
          "Number of poses is not equal to number of time stamps. Not adding "
          "trajectory measurement.");
    } else {
      std::vector<Eigen::Matrix4d, pose_allocator> poses;
      std::vector<ros::Time> stamps;
      for (int i = 0; i < num_poses; i++) {
        std::vector<float> current_pose;
        for (int j = 0; j < 12; j++) {
          current_pose.push_back(traj_measurement.poses[12 * i + j]);
        }
        Eigen::Matrix4d T_KEYFRAME_FRAME = VectorToTransform(current_pose);

        poses.push_back(T_KEYFRAME_FRAME);
        stamps.push_back(ros::Time(traj_measurement.stamps[i]));
      }

      submaps_[submap_id].AddTrajectoryMeasurement(poses, stamps, stamp);
    }
  }
  return new_transaction;
}

int GlobalMap::GetSubmapId(const Eigen::Matrix4d& T_WORLD_BASELINK) {
  // check if current pose is within "submap_size" from previous submap, or
  // current submap. We prioritize the previous submap for the case where data
  // isn't coming in in order (e.g., lidar data may come in slower)

  // first check if submaps is empty
  if (submaps_.empty()) {
    return 0;
  }

  Eigen::Vector3d t_WORLD_FRAME = T_WORLD_BASELINK.block(0, 3, 3, 1);

  Eigen::Vector3d t_WORLD_SUBMAPCUR =
      submaps_.at(submaps_.size() - 1).T_WORLD_SUBMAP_INIT().block(0, 3, 3, 1);

  // if only one submap exists, we only check the pose is within this first
  // submap
  if (submaps_.size() == 1) {
    if ((t_WORLD_FRAME - t_WORLD_SUBMAPCUR).norm() < params_.submap_size) {
      return 0;
    } else {
      return 1;
    }
  }

  // otherwise, also check the prev submap and prioritize that one
  Eigen::Vector3d t_WORLD_SUBMAPPREV =
      submaps_.at(submaps_.size() - 2).T_WORLD_SUBMAP_INIT().block(0, 3, 3, 1);

  if ((t_WORLD_FRAME - t_WORLD_SUBMAPPREV).norm() < params_.submap_size) {
    return submaps_.size() - 2;
  } else if ((t_WORLD_FRAME - t_WORLD_SUBMAPCUR).norm() < params_.submap_size) {
    return submaps_.size() - 1;
  } else {
    return submaps_.size();
  }
}

fuse_core::Transaction::SharedPtr GlobalMap::InitiateNewSubmapPose() {
  const Submap& current_submap = submaps_[submaps_.size() - 1];
  const Submap& previous_submap = submaps_[submaps_.size() - 2];
  bs_constraints::frame_to_frame::Pose3DStampedTransaction new_transaction(
      current_submap.Stamp());

  new_transaction.AddPoseVariables(current_submap.Position(),
                                   current_submap.Orientation(),
                                   current_submap.Stamp());

  Eigen::Matrix4d T_PREVIOUS_CURRENT =
      beam::InvertTransform(previous_submap.T_WORLD_SUBMAP()) *
      current_submap.T_WORLD_SUBMAP();

  std::string source = "LOCALMAPPER";
  new_transaction.AddPoseConstraint(
      previous_submap.T_WORLD_SUBMAP(), current_submap.T_WORLD_SUBMAP(),
      previous_submap.Stamp(), current_submap.Stamp(), T_PREVIOUS_CURRENT,
      params_.local_mapper_covariance, source);

  return new_transaction.GetTransaction();
}

fuse_core::Transaction::SharedPtr GlobalMap::FindLoopClosures() {
  int current_index = submaps_.size() - 2;
  std::vector<int> matched_indices;
  std::vector<Eigen::Matrix4d, pose_allocator> Ts_MATCH_QUERY;
  loop_closure_candidate_search_->FindLoopClosureCandidates(
      submaps_, current_index, matched_indices, Ts_MATCH_QUERY);

  if (matched_indices.size() == 0) {
    return nullptr;
  }

  fuse_core::Transaction::SharedPtr transaction =
      std::make_shared<fuse_core::Transaction>();
  for (int i = 0; i < matched_indices.size(); i++) {
    fuse_core::Transaction::SharedPtr new_transaction =
        loop_closure_refinement_->GenerateTransaction(
            submaps_[matched_indices[i]], submaps_[current_index],
            Ts_MATCH_QUERY[i]);
    transaction->merge(*new_transaction);
  }

  // TODO: Implement this
  // SendSubmap(best_index);
  return transaction;
}

void GlobalMap::UpdateSubmapPoses(fuse_core::Graph::ConstSharedPtr graph_msg) {
  for (auto& submap : submaps_) {
    submap.UpdatePose(graph_msg);
  }
}

void GlobalMap::SaveLidarSubmaps(const std::string& output_path,
                                 bool save_initial) {
  if (!boost::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path, not saving submaps. Input: {}",
               output_path);
  }

  // save optimized submap
  std::string submaps_path = output_path + "lidar_submaps_optimized/";
  boost::filesystem::create_directory(submaps_path);
  for (int i = 0; i < submaps_.size(); i++) {
    std::string submap_name =
        submaps_path + "lidar_submap" + std::to_string(i) + ".pcd";
    submaps_[i].SaveLidarMapInWorldFrame(submap_name, max_output_map_size_);
  }

  if (!save_initial) {
    return;
  }

  // save initial submap
  std::string submaps_path_initial = output_path + "lidar_submaps_initial/";
  boost::filesystem::create_directory(submaps_path_initial);
  for (int i = 0; i < submaps_.size(); i++) {
    std::string submap_name =
        submaps_path_initial + "lidar_submap" + std::to_string(i) + ".pcd";
    submaps_[i].SaveLidarMapInWorldFrame(submap_name, max_output_map_size_,
                                         true);
  }
}

void GlobalMap::SaveKeypointSubmaps(const std::string& output_path,
                                    bool save_initial) {
  if (!boost::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path, not saving submaps. Input: {}",
               output_path);
  }

  // save optimized submaps
  std::string submaps_path = output_path + "keypoint_submaps_optimized/";
  boost::filesystem::create_directory(submaps_path);
  for (int i = 0; i < submaps_.size(); i++) {
    std::string submap_name =
        submaps_path + "keypoint_submap" + std::to_string(i) + ".pcd";
    submaps_[i].SaveKeypointsMapInWorldFrame(submap_name);
  }

  if (!save_initial) {
    return;
  }

  // save initial submaps
  std::string submaps_path_initial = output_path + "keypoint_submaps_initial/";
  boost::filesystem::create_directory(submaps_path_initial);
  for (int i = 0; i < submaps_.size(); i++) {
    std::string submap_name =
        submaps_path_initial + "keypoint_submap" + std::to_string(i) + ".pcd";
    submaps_[i].SaveKeypointsMapInWorldFrame(submap_name, true);
  }
}

void GlobalMap::SaveTrajectoryFile(const std::string& output_path,
                                   bool save_initial) {
  std::string date = beam::ConvertTimeToDate(std::chrono::system_clock::now());

  // Get trajectory
  beam_mapping::Poses poses;
  poses.SetPoseFileDate(date);
  poses.SetFixedFrame(extrinsics_.GetWorldFrameId());
  poses.SetMovingFrame(extrinsics_.GetBaselinkFrameId());
  for (auto& submap : submaps_) {
    Eigen::Matrix4d T_WORLD_SUBMAP = submap.T_WORLD_SUBMAP();
    for (auto& pose_stamped : submap.GetTrajectory()) {
      Eigen::Matrix4d& T_SUBMAP_BASELINK = pose_stamped.pose;
      Eigen::Matrix4d T_WORLD_BASELINK = T_WORLD_SUBMAP * T_SUBMAP_BASELINK;
      poses.AddSingleTimeStamp(pose_stamped.stamp);
      poses.AddSinglePose(Eigen::Affine3d(T_WORLD_BASELINK));
    }
  }

  // save
  std::string output_file =
      output_path + "global_map_trajectory_optimized.json";
  poses.WriteToJSON(output_file);

  if (!save_initial) {
    return;
  }

  // Get trajectory
  beam_mapping::Poses poses_initial;
  poses_initial.SetPoseFileDate(date);
  poses_initial.SetFixedFrame(extrinsics_.GetWorldFrameId());
  poses_initial.SetMovingFrame(extrinsics_.GetBaselinkFrameId());
  for (auto& submap : submaps_) {
    Eigen::Matrix4d T_WORLD_SUBMAP = submap.T_WORLD_SUBMAP_INIT();
    for (auto& pose_stamped : submap.GetTrajectory()) {
      Eigen::Matrix4d& T_SUBMAP_BASELINK = pose_stamped.pose;
      Eigen::Matrix4d T_WORLD_BASELINK = T_WORLD_SUBMAP * T_SUBMAP_BASELINK;
      poses_initial.AddSingleTimeStamp(pose_stamped.stamp);
      poses_initial.AddSinglePose(Eigen::Affine3d(T_WORLD_BASELINK));
    }
  }

  // save
  std::string output_file_initial =
      output_path + "global_map_trajectory_initial.json";
  poses_initial.WriteToJSON(output_file_initial);
}

void GlobalMap::SaveTrajectoryClouds(const std::string& output_path,
                                     bool save_initial) {
  // get trajectory
  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  for (auto& submap : submaps_) {
    Eigen::Matrix4d T_WORLD_SUBMAP = submap.T_WORLD_SUBMAP();
    std::vector<Submap::PoseStamped> poses_stamped = submap.GetTrajectory();
    for (const Submap::PoseStamped& pose_stamped : poses_stamped) {
      const Eigen::Matrix4d& T_SUBMAP_BASELINK = pose_stamped.pose;
      Eigen::Vector4d p(0, 0, 0, 1);
      p = T_WORLD_SUBMAP * T_SUBMAP_BASELINK * p;
      pcl::PointXYZRGBL point;
      point.x = p[0];
      point.y = p[1];
      point.z = p[2];
      point.label = pose_stamped.stamp.toSec();
      cloud.push_back(point);
    }
  }

  if (!cloud.empty()) {
    std::string output_file =
        output_path + "global_map_trajectory_optimized.pcd";
    BEAM_INFO("Saving trajectory cloud to: {}", output_file);
    pcl::io::savePCDFileASCII(output_file, cloud);
  } else {
    BEAM_WARN("Trajectory cloud empty, not saving poses cloud.");
  }

  if (!save_initial) {
    return;
  }

  // get trajectory initial
  pcl::PointCloud<pcl::PointXYZRGBL> cloud_initial;
  for (auto& submap : submaps_) {
    Eigen::Matrix4d T_WORLD_SUBMAP = submap.T_WORLD_SUBMAP_INIT();
    auto poses_stamped = submap.GetTrajectory();
    for (const Submap::PoseStamped& pose_stamped : poses_stamped) {
      Eigen::Vector4d p(0, 0, 0, 1);
      const Eigen::Matrix4d& T_SUBMAP_BASELINK = pose_stamped.pose;
      p = T_WORLD_SUBMAP * T_SUBMAP_BASELINK * p;
      pcl::PointXYZRGBL point;
      point.x = p[0];
      point.y = p[1];
      point.z = p[2];
      point.label = pose_stamped.stamp.toSec();
      cloud_initial.push_back(point);
    }
  }

  if (!cloud_initial.empty()) {
    std::string output_file_initial =
        output_path + "global_map_trajectory_initial.pcd";
    BEAM_INFO("Saving trajectory cloud to: {}", output_file_initial);
    pcl::io::savePCDFileASCII(output_file_initial, cloud_initial);
  } else {
    BEAM_WARN("Trajectory cloud empty, not saving poses cloud.");
  }
}

void GlobalMap::SaveSubmapFrames(const std::string& output_path,
                                 bool save_initial) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  for (auto& submap : submaps_) {
    pcl::PointCloud<pcl::PointXYZRGBL> frame =
        beam::CreateFrameCol(submap.Stamp());
    pcl::PointCloud<pcl::PointXYZRGBL> frame_transformed;
    pcl::transformPointCloud(frame, frame_transformed, submap.T_WORLD_SUBMAP());
    cloud += frame_transformed;
  }
  std::string output_file =
      output_path + "global_map_submap_poses_optimized.pcd";
  BEAM_INFO("Saving submap frames to: {}", output_file);
  pcl::io::savePCDFileASCII(output_file, cloud);

  if (!save_initial) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZRGBL> cloud_initial;
  for (auto& submap : submaps_) {
    pcl::PointCloud<pcl::PointXYZRGBL> frame =
        beam::CreateFrameCol(submap.Stamp());
    pcl::PointCloud<pcl::PointXYZRGBL> frame_transformed;
    pcl::transformPointCloud(frame, frame_transformed,
                             submap.T_WORLD_SUBMAP_INIT());
    cloud_initial += frame_transformed;
  }
  std::string output_file_initial =
      output_path + "global_map_submap_poses_initial.pcd";
  BEAM_INFO("Saving submap frames to: {}", output_file_initial);
  pcl::io::savePCDFileASCII(output_file_initial, cloud_initial);
}

}  // namespace global_mapping

}  // namespace bs_models
