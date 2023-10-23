#include <bs_models/global_mapping/global_map.h>

#include <chrono>
#include <ctime>
#include <filesystem>

#include <pcl/conversions.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/descriptors/Descriptor.h>
#include <beam_mapping/Poses.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>
#include <beam_utils/pcl_conversions.h>
#include <beam_utils/pointclouds.h>
#include <beam_utils/time.h>

#include <bs_common/conversions.h>
#include <bs_common/graph_access.h>
#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>
#include <bs_models/reloc/reloc_methods.h>

namespace bs_models { namespace global_mapping {

using namespace reloc;

GlobalMap::Params::Params() {
  double local_map_cov_diag = 1e-3;
  double loop_cov_diag = 1e-5;

  // clang-format off
  local_mapper_covariance << local_map_cov_diag, 0, 0, 0, 0, 0,
                             0, local_map_cov_diag, 0, 0, 0, 0,
                             0, 0, local_map_cov_diag, 0, 0, 0,
                             0, 0, 0, local_map_cov_diag, 0, 0,
                             0, 0, 0, 0, local_map_cov_diag, 0,
                             0, 0, 0, 0, 0, local_map_cov_diag;

  loop_closure_covariance << loop_cov_diag, 0, 0, 0, 0, 0,
                      0, loop_cov_diag, 0, 0, 0, 0,
                      0, 0, loop_cov_diag, 0, 0, 0,
                      0, 0, 0, loop_cov_diag, 0, 0,
                      0, 0, 0, 0, loop_cov_diag, 0,
                      0, 0, 0, 0, 0, loop_cov_diag;
  // clang-format on
}

void GlobalMap::Params::LoadJson(const std::string& config_path) {
  if (config_path.empty()) {
    BEAM_INFO(
        "No config file provided to global map, using default parameters.");
    return;
  }

  BEAM_INFO("Loading global map config file: {}", config_path);

  nlohmann::json J;
  if (!beam::ReadJson(config_path, J)) {
    BEAM_ERROR("Unable to read global map config");
    throw std::runtime_error{"Unable to read global map config"};
  }

  bs_common::ValidateJsonKeysOrThrow(
      std::vector<std::string>{
          "submap_size_m", "loop_closure_candidate_search_config",
          "loop_closure_refinement_config", "local_mapper_covariance_diag",
          "loop_closure_covariance_diag", "publishing"},
      J);

  submap_size = J["submap_size_m"];

  std::string loop_closure_candidate_search_config_rel =
      J["loop_closure_candidate_search_config"];
  if (!loop_closure_candidate_search_config_rel.empty()) {
    loop_closure_candidate_search_config =
        beam::CombinePaths(bs_common::GetBeamSlamConfigPath(),
                           loop_closure_candidate_search_config_rel);
  }

  std::string loop_closure_refinement_config_rel =
      J["loop_closure_refinement_config"];
  if (!loop_closure_refinement_config_rel.empty()) {
    loop_closure_refinement_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), loop_closure_refinement_config_rel);
  }

  std::vector<double> vec = J["local_mapper_covariance_diag"];
  if (vec.size() != 6) {
    BEAM_ERROR(
        "Invalid local mapper covariance diagonal (6 values required). Using "
        "default.");
  } else {
    Eigen::VectorXd vec_eig(6);
    vec_eig << vec[0], vec[1], vec[2], vec[3], vec[4], vec[5];
    local_mapper_covariance = vec_eig.asDiagonal();
  }

  std::vector<double> vec2 = J["loop_closure_covariance_diag"];
  if (vec2.size() != 6) {
    BEAM_ERROR(
        "Invalid loop closure covariance diagonal (6 values required). Using "
        "default.");
  } else {
    Eigen::VectorXd vec_eig = Eigen::VectorXd(6);
    vec_eig << vec2[0], vec2[1], vec2[2], vec2[3], vec2[4], vec2[5];
    loop_closure_covariance = vec_eig.asDiagonal();
  }

  // load filters
  nlohmann::json J_publishing = J["publishing"];

  bs_common::ValidateJsonKeysOrThrow(
      std::vector<std::string>{"submap_lidar_filters",
                               "globalmap_lidar_filters"},
      J_publishing);

  nlohmann::json J_submap_filters = J_publishing["submap_lidar_filters"];
  nlohmann::json J_globalmap_filters = J_publishing["globalmap_lidar_filters"];

  ros_submap_filter_params =
      beam_filtering::LoadFilterParamsVector(J_submap_filters);
  ros_globalmap_filter_params =
      beam_filtering::LoadFilterParamsVector(J_globalmap_filters);
}

void GlobalMap::Params::SaveJson(const std::string& filename) {
  std::string config_path = bs_common::GetBeamSlamConfigPath();
  int config_len = config_path.size();
  std::string loop_closure_candidate_search_config_rel =
      loop_closure_candidate_search_config.substr(
          config_len, loop_closure_candidate_search_config.size() - config_len);
  std::string loop_closure_refinement_config_rel =
      loop_closure_refinement_config.substr(
          config_len, loop_closure_refinement_config.size() - config_len);

  nlohmann::json J = {
      {"submap_size_m", submap_size},
      {"loop_closure_candidate_search_config",
       loop_closure_candidate_search_config_rel},
      {"loop_closure_refinement_config", loop_closure_refinement_config_rel},
      {"local_mapper_covariance_diag",
       {local_mapper_covariance(0, 0), local_mapper_covariance(1, 1),
        local_mapper_covariance(2, 2), local_mapper_covariance(3, 3),
        local_mapper_covariance(4, 4), local_mapper_covariance(5, 5)}},
      {"loop_closure_covariance_diag",
       {loop_closure_covariance(0, 0), loop_closure_covariance(1, 1),
        loop_closure_covariance(2, 2), loop_closure_covariance(3, 3),
        loop_closure_covariance(4, 4), loop_closure_covariance(5, 5)}}};

  std::ofstream file(filename);
  file << std::setw(4) << J << std::endl;
}

GlobalMap::GlobalMap(
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
    const std::shared_ptr<bs_common::ExtrinsicsLookupBase>& extrinsics)
    : camera_model_(camera_model), extrinsics_(extrinsics) {
  Setup();
}

GlobalMap::GlobalMap(
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
    const std::shared_ptr<bs_common::ExtrinsicsLookupBase>& extrinsics,
    const Params& params)
    : camera_model_(camera_model), params_(params), extrinsics_(extrinsics) {
  Setup();
}

GlobalMap::GlobalMap(
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
    const std::shared_ptr<bs_common::ExtrinsicsLookupBase>& extrinsics,
    const std::string& config_path)
    : camera_model_(camera_model), extrinsics_(extrinsics) {
  params_.LoadJson(config_path);
  Setup();
}

GlobalMap::GlobalMap(const std::string& data_root_directory) {
  if (!Load(data_root_directory)) {
    BEAM_ERROR("Unable to load global map data, check data content/formats.");
    throw std::runtime_error{
        "Unable to instantiate GlobalMap with input data."};
  }
}

std::vector<SubmapPtr> GlobalMap::GetSubmaps() {
  return submaps_;
}

void GlobalMap::SetSubmaps(std::vector<SubmapPtr>& submaps) {
  submaps_ = submaps;
}

void GlobalMap::SetStoreNewSubmaps(bool store_new_submaps) {
  store_newly_completed_submaps_ = store_new_submaps;
}

void GlobalMap::SetStoreNewScans(bool store_new_scans) {
  store_new_scans_ = store_new_scans;
}

void GlobalMap::SetStoreUpdatedGlobalMap(bool store_updated_global_map) {
  store_updated_global_map_ = store_updated_global_map;
}

void GlobalMap::SetLoopClosureResultsPath(const std::string& output_path) {
  loop_closure_results_path_ = output_path;
}

std::vector<std::shared_ptr<RosMap>> GlobalMap::GetRosMaps() {
  std::vector<std::shared_ptr<RosMap>> maps_vector;
  while (!ros_new_scans_.empty()) {
    maps_vector.push_back(ros_new_scans_.front());
    ros_new_scans_.pop();
  }
  while (!ros_submaps_.empty()) {
    maps_vector.push_back(ros_submaps_.front());
    ros_submaps_.pop();
  }
  if (ros_global_lidar_map_ != nullptr) {
    maps_vector.push_back(ros_global_lidar_map_);
    ros_global_lidar_map_ = nullptr;
  }
  if (ros_global_keypoints_map_ != nullptr) {
    maps_vector.push_back(ros_global_keypoints_map_);
    ros_global_keypoints_map_ = nullptr;
  }

  return maps_vector;
}

void GlobalMap::Setup() {
  // initiate loop_closure candidate search
  loop_closure_candidate_search_ = reloc::RelocCandidateSearchBase::Create(
      params_.loop_closure_candidate_search_config);

  // initiate loop_closure refinement
  loop_closure_refinement_ = reloc::RelocRefinementBase::Create(
      params_.loop_closure_refinement_config);
}

fuse_core::Transaction::SharedPtr GlobalMap::AddMeasurement(
    const bs_common::CameraMeasurementMsg& cam_measurement,
    const bs_common::LidarMeasurementMsg& lid_measurement,
    const nav_msgs::Path& traj_measurement,
    const Eigen::Matrix4d& T_WORLD_BASELINK, const ros::Time& stamp) {
  fuse_core::Transaction::SharedPtr new_transaction = nullptr;

  int submap_id = GetSubmapId(T_WORLD_BASELINK);

  // if id is equal to submap size then we need to create a new submap
  if (submap_id == submaps_.size()) {
    SubmapPtr new_submap = std::make_shared<Submap>(stamp, T_WORLD_BASELINK,
                                                    camera_model_, extrinsics_);
    submaps_.push_back(new_submap);
    new_transaction = InitiateNewSubmapPose();

    // Run loop closure on the previously completed submap. Current submap is
    // size -1, therefore the last is size - 2
    fuse_core::Transaction::SharedPtr loop_closure_transaction =
        RunLoopClosure(submaps_.size() - 2);

    if (loop_closure_transaction != nullptr) {
      new_transaction->merge(*loop_closure_transaction);
    }

    if (store_newly_completed_submaps_ && submaps_.size() > 1) {
      AddRosSubmap(submaps_.size() - 2);
    }
  }

  // add camera measurement if not empty
  if (!cam_measurement.landmarks.empty()) {
    ROS_DEBUG("Adding camera measurement to global map.");
    submaps_.at(submap_id)->AddCameraMeasurement(cam_measurement,
                                                 T_WORLD_BASELINK);
  }

  // if lidar measurement exists, check frame id
  if (!lid_measurement.lidar_points.empty() ||
      !lid_measurement.lidar_edges_strong.empty() ||
      !lid_measurement.lidar_surfaces_strong.empty()) {
    if (lid_measurement.frame_id != extrinsics_->GetLidarFrameId()) {
      BEAM_WARN(
          "Lidar measurement frame id not consistent with lidar frame in the "
          "extrinsics class.");
    }
  }

  // add lidar measurement if not empty
  // ROS_DEBUG("Adding lidar measurement to global map.");
  PointCloud cloud;
  if (!lid_measurement.lidar_points.empty()) {
    cloud = beam::ROSVectorToPCL(lid_measurement.lidar_points);

    // add ros msg if applicable
    if (store_new_scans_) { AddNewRosScan(cloud, T_WORLD_BASELINK, stamp); }

    submaps_.at(submap_id)->AddLidarMeasurement(cloud, T_WORLD_BASELINK, stamp);
  }
  if (lid_measurement.lidar_edges_strong.size() +
          lid_measurement.lidar_edges_strong.size() +
          lid_measurement.lidar_surfaces_strong.size() +
          lid_measurement.lidar_surfaces_weak.size() >
      0) {
    beam_matching::LoamPointCloud loamCloud;
    loamCloud.edges.strong.cloud =
        beam::ROSVectorToPCLIRT(lid_measurement.lidar_edges_strong);
    loamCloud.edges.weak.cloud =
        beam::ROSVectorToPCLIRT(lid_measurement.lidar_edges_weak);
    loamCloud.surfaces.strong.cloud =
        beam::ROSVectorToPCLIRT(lid_measurement.lidar_surfaces_strong);
    loamCloud.surfaces.weak.cloud =
        beam::ROSVectorToPCLIRT(lid_measurement.lidar_surfaces_weak);
    submaps_.at(submap_id)->AddLidarMeasurement(loamCloud, T_WORLD_BASELINK,
                                                stamp);
  }

  // add trajectory measurement if not empty
  if (!traj_measurement.poses.empty()) {
    ROS_DEBUG("Adding trajectory measurement to global map.");
    std::vector<Eigen::Matrix4d, beam::AlignMat4d> poses;
    std::vector<ros::Time> stamps;
    for (const auto& pose : traj_measurement.poses) {
      Eigen::Matrix4d T_KEYFRAME_FRAME;
      bs_common::PoseMsgToTransformationMatrix(pose, T_KEYFRAME_FRAME);
      poses.push_back(T_KEYFRAME_FRAME);
      stamps.push_back(pose.header.stamp);
    }
  }

  return new_transaction;
}

fuse_core::Transaction::SharedPtr GlobalMap::TriggerLoopClosure() {
  if (submaps_.size() < 2) { return nullptr; }

  return RunLoopClosure(submaps_.size() - 1);
}

int GlobalMap::GetSubmapId(const Eigen::Matrix4d& T_WORLD_BASELINK) {
  // check if current pose is within "submap_size" from previous submap, or
  // current submap. We prioritize the previous submap for the case where data
  // isn't coming in in order (e.g., lidar data may come in slower)

  // first check if submaps is empty
  if (submaps_.empty()) { return 0; }

  Eigen::Vector3d t_WORLD_FRAME = T_WORLD_BASELINK.block(0, 3, 3, 1);

  Eigen::Vector3d t_WORLD_SUBMAPCUR =
      submaps_.at(submaps_.size() - 1)->T_WORLD_SUBMAP_INIT().block(0, 3, 3, 1);

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
      submaps_.at(submaps_.size() - 2)->T_WORLD_SUBMAP_INIT().block(0, 3, 3, 1);

  if ((t_WORLD_FRAME - t_WORLD_SUBMAPPREV).norm() < params_.submap_size) {
    return submaps_.size() - 2;
  } else if ((t_WORLD_FRAME - t_WORLD_SUBMAPCUR).norm() < params_.submap_size) {
    return submaps_.size() - 1;
  } else {
    return submaps_.size();
  }
}

fuse_core::Transaction::SharedPtr GlobalMap::InitiateNewSubmapPose() {
  ROS_DEBUG("Initiating new submap pose");

  const SubmapPtr& current_submap = submaps_.at(submaps_.size() - 1);
  bs_constraints::Pose3DStampedTransaction new_transaction(
      current_submap->Stamp());
  new_transaction.AddPoseVariables(current_submap->Position(),
                                   current_submap->Orientation(),
                                   current_submap->Stamp());

  // if first submap, add prior then return
  if (submaps_.size() == 1) {
    new_transaction.AddPosePrior(
        current_submap->Position(), current_submap->Orientation(),
        pose_prior_noise_, "GlobalMap::InitiateNewSubmapPose");
    return new_transaction.GetTransaction();
  }

  // If not first submap add constraint to previous
  const SubmapPtr& previous_submap = submaps_.at(submaps_.size() - 2);

  Eigen::Matrix4d T_PREVIOUS_CURRENT =
      beam::InvertTransform(previous_submap->T_WORLD_SUBMAP()) *
      current_submap->T_WORLD_SUBMAP();
  new_transaction.AddPoseConstraint(
      previous_submap->Position(), current_submap->Position(),
      previous_submap->Orientation(), current_submap->Orientation(),
      bs_common::TransformMatrixToVectorWithQuaternion(T_PREVIOUS_CURRENT),
      params_.local_mapper_covariance, "GlobalMap::InitiateNewSubmapPose");

  ROS_DEBUG("Returning submap pose prior");
  return new_transaction.GetTransaction();
}

fuse_core::Transaction::SharedPtr GlobalMap::RunLoopClosure(int query_index) {
  // if first submap, don't look for loop_closures
  if (submaps_.size() == 1) { return nullptr; }

  ROS_DEBUG("Searching for loop closure candidates");
  const Eigen::Matrix4d& T_WORLD_QUERY =
      submaps_.at(query_index)->T_WORLD_SUBMAP();

  std::vector<int> matched_indices;
  std::vector<Eigen::Matrix4d, beam::AlignMat4d> Ts_MATCH_QUERY;
  // ignore the current empty submap, and the last full submap (the query)
  static int ignore_last_n_submaps = 2;
  loop_closure_candidate_search_->FindRelocCandidates(
      submaps_, T_WORLD_QUERY, matched_indices, Ts_MATCH_QUERY,
      ignore_last_n_submaps);

  // remove candidate if it is equal to the query submap, or one before
  std::vector<int> matched_indices_filtered;
  for (int i : matched_indices) {
    if (i == query_index || i == query_index - 1) { continue; }
    matched_indices_filtered.push_back(i);
  }

  ROS_DEBUG("Found %zu loop closure candidates.",
            matched_indices_filtered.size());

  if (matched_indices_filtered.size() == 0) { return nullptr; }

  ROS_DEBUG("Matched index[0]: %d, Query Index: %d, No. or submaps: %zu. "
            "Running loop "
            "closure refinement",
            matched_indices_filtered.at(0), query_index, submaps_.size());

  fuse_core::Transaction::SharedPtr transaction =
      std::make_shared<fuse_core::Transaction>();
  for (int i = 0; i < matched_indices_filtered.size(); i++) {
    // if the matched index is adjacent to the query index, ignore it. This
    // would happen from improper candidate search implementations
    if (matched_indices_filtered[i] == query_index + 1 ||
        matched_indices_filtered[i] == query_index - 1) {
      continue;
    }

    const auto& matched_submap = submaps_.at(matched_indices_filtered[i]);
    const auto& query_submap = submaps_.at(query_index);
    RelocRefinementResults results = loop_closure_refinement_->RunRefinement(
        matched_submap, query_submap, Ts_MATCH_QUERY[i],
        loop_closure_results_path_);

    if (!results.successful) { continue; }

    bs_constraints::Pose3DStampedTransaction new_transaction(
        query_submap->Stamp());
    new_transaction.AddPoseConstraint(
        matched_submap->Position(), query_submap->Position(),
        matched_submap->Orientation(), query_submap->Orientation(),
        bs_common::TransformMatrixToVectorWithQuaternion(results.T_MATCH_QUERY),
        params_.loop_closure_covariance, "GlobalMap::RunLoopClosure");

    transaction->merge(*(new_transaction.GetTransaction()));
  }

  int num_constraints = bs_common::GetNumberOfConstraints(transaction);
  ROS_DEBUG("Returning %d loop closure transactions", num_constraints);
  return transaction;
}

void GlobalMap::UpdateSubmapPoses(fuse_core::Graph::ConstSharedPtr graph_msg,
                                  const ros::Time& update_time) {
  last_update_time_ = update_time;

  for (uint16_t i = 0; i < submaps_.size(); i++) {
    submaps_.at(i)->UpdatePose(graph_msg);
  }

  if (store_updated_global_map_) { AddRosGlobalMap(); }

  global_map_updates_++;
}

void GlobalMap::SaveData(const std::string& output_path) {
  if (!std::filesystem::exists(output_path)) {
    BEAM_ERROR(
        "Global map output path does not exist, not saving map. Input: {}",
        output_path);
    return;
  }

  BEAM_INFO("Saving full global map to: {}", output_path);
  params_.SaveJson(beam::CombinePaths(output_path, "params.json"));
  camera_model_->WriteJSON(
      beam::CombinePaths(output_path, "camera_model.json"));
  extrinsics_->SaveExtrinsicsToJson(
      beam::CombinePaths(output_path, "extrinsics.json"));
  extrinsics_->SaveFrameIdsToJson(
      beam::CombinePaths(output_path, "frame_ids.json"));
  for (uint16_t i = 0; i < submaps_.size(); i++) {
    std::string submap_dir =
        beam::CombinePaths(output_path, "submap" + std::to_string(i));
    std::filesystem::create_directory(submap_dir);
    submaps_.at(i)->SaveData(submap_dir);
  }
  BEAM_INFO("Done saving global map.");
}

bool GlobalMap::Load(const std::string& root_directory) {
  if (!std::filesystem::exists(root_directory)) {
    BEAM_ERROR(
        "Global map root directory path does not exist, not loading map. "
        "Input: {}",
        root_directory);
    return false;
  }
  BEAM_INFO("Loading full global map from: {}", root_directory);

  // load params
  std::string params_path = beam::CombinePaths(root_directory, "params.json");
  if (!std::filesystem::exists(params_path)) {
    BEAM_ERROR(
        "params.json not foundn in root directory, not loading GlobalMap. "
        "Input root directory: {}",
        root_directory);
    return false;
  }
  params_.LoadJson(params_path);

  // load camera model
  std::string camera_model_path =
      beam::CombinePaths(root_directory, "camera_model.json");
  if (!std::filesystem::exists(camera_model_path)) {
    BEAM_ERROR(
        "camera_model.json not found in root directory, not loading GlobalMap. "
        "Input root directory: {}",
        root_directory);
    return false;
  }

  BEAM_INFO("Loading camera model from: {}", camera_model_path);
  camera_model_ = beam_calibration::CameraModel::Create(camera_model_path);

  // load extrinsics
  std::string frame_ids_path =
      beam::CombinePaths(root_directory, "frame_ids.json");
  std::string extrinsics_path =
      beam::CombinePaths(root_directory, "extrinsics.json");
  extrinsics_ = std::make_shared<bs_common::ExtrinsicsLookupBase>(
      frame_ids_path, extrinsics_path);

  // setup general stuff
  Setup();

  int submap_num = 0;
  while (true) {
    std::string submap_dir = beam::CombinePaths(
        root_directory, "submap" + std::to_string(submap_num));
    if (!std::filesystem::exists(submap_dir)) { break; }

    SubmapPtr current_submap = std::make_shared<Submap>(
        ros::Time(0), Eigen::Matrix4d::Identity(), camera_model_, extrinsics_);
    BEAM_INFO("Loading submap from: {}", submap_dir);
    current_submap->LoadData(submap_dir, false);
    submaps_.push_back(current_submap);
    submap_num++;
  }

  if (submap_num == 0) {
    BEAM_ERROR("No submaps loaded, root directory empty.");
    return false;
  } else {
    BEAM_INFO("Done loading global map. Loaded {} submaps.", submap_num);
    return true;
  }
}

void GlobalMap::SaveLidarSubmaps(const std::string& output_path,
                                 bool save_initial) {
  if (!std::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path, not saving submaps. Input: {}",
               output_path);
    return;
  }

  // save optimized submap
  std::string submaps_path =
      beam::CombinePaths(output_path, "lidar_submaps_optimized");
  std::filesystem::create_directory(submaps_path);
  for (int i = 0; i < submaps_.size(); i++) {
    std::string submap_path = beam::CombinePaths(
        submaps_path, "lidar_submap" + std::to_string(i) + ".pcd");
    submaps_.at(i)->SaveLidarMapInWorldFrame(submap_path, max_output_map_size_);
  }

  if (!save_initial) { return; }

  // save initial submap
  std::string submaps_path_initial =
      beam::CombinePaths(output_path, "lidar_submaps_initial");
  std::filesystem::create_directory(submaps_path_initial);
  for (int i = 0; i < submaps_.size(); i++) {
    std::string submap_path = beam::CombinePaths(
        submaps_path_initial, "lidar_submap" + std::to_string(i) + ".pcd");
    submaps_.at(i)->SaveLidarMapInWorldFrame(submap_path, max_output_map_size_,
                                             true);
  }
}

void GlobalMap::SaveKeypointSubmaps(const std::string& output_path,
                                    bool save_initial) {
  if (!std::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path, not saving submaps. Input: {}",
               output_path);
    return;
  }

  // save optimized submaps
  std::string submaps_path =
      beam::CombinePaths(output_path, "keypoint_submaps_optimized");
  std::filesystem::create_directory(submaps_path);
  for (int i = 0; i < submaps_.size(); i++) {
    std::string submap_path = beam::CombinePaths(
        submaps_path, "keypoint_submap" + std::to_string(i) + ".pcd");
    submaps_.at(i)->SaveKeypointsMapInWorldFrame(submap_path);
  }

  if (!save_initial) { return; }

  // save initial submaps
  std::string submaps_path_initial =
      beam::CombinePaths(output_path, "keypoint_submaps_initial");
  std::filesystem::create_directory(submaps_path_initial);
  for (int i = 0; i < submaps_.size(); i++) {
    std::string submap_path = beam::CombinePaths(
        submaps_path_initial, "keypoint_submap" + std::to_string(i) + ".pcd");
    submaps_.at(i)->SaveKeypointsMapInWorldFrame(submap_path, true);
  }
}

void GlobalMap::SaveTrajectoryFile(const std::string& output_path,
                                   bool save_initial) {
  if (!std::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path, not saving trajectory file. Input: {}",
               output_path);
    return;
  }

  std::string date = beam::ConvertTimeToDate(std::chrono::system_clock::now());

  // Get trajectory
  beam_mapping::Poses poses;
  poses.SetPoseFileDate(date);
  poses.SetFixedFrame(extrinsics_->GetWorldFrameId());
  poses.SetMovingFrame(extrinsics_->GetBaselinkFrameId());
  for (uint16_t i = 0; i < submaps_.size(); i++) {
    const SubmapPtr& submap = submaps_.at(i);
    Eigen::Matrix4d T_WORLD_SUBMAP = submap->T_WORLD_SUBMAP();
    auto poses_stamped = submap->GetTrajectory();
    for (auto& pose_stamped : poses_stamped) {
      Eigen::Matrix4d& T_SUBMAP_BASELINK = pose_stamped.pose;
      Eigen::Matrix4d T_WORLD_BASELINK = T_WORLD_SUBMAP * T_SUBMAP_BASELINK;
      poses.AddSingleTimeStamp(pose_stamped.stamp);
      poses.AddSinglePose(T_WORLD_BASELINK);
    }
  }

  // save
  std::string output_file =
      beam::CombinePaths(output_path, "global_map_trajectory_optimized.json");
  poses.WriteToJSON(output_file);

  if (!save_initial) { return; }

  // Get trajectory
  beam_mapping::Poses poses_initial;
  poses_initial.SetPoseFileDate(date);
  poses_initial.SetFixedFrame(extrinsics_->GetWorldFrameId());
  poses_initial.SetMovingFrame(extrinsics_->GetBaselinkFrameId());
  for (uint16_t i = 0; i < submaps_.size(); i++) {
    const SubmapPtr& submap = submaps_.at(i);
    Eigen::Matrix4d T_WORLD_SUBMAP = submap->T_WORLD_SUBMAP_INIT();
    std::vector<Submap::PoseStamped> poses_stamped = submap->GetTrajectory();
    for (auto& pose_stamped : poses_stamped) {
      Eigen::Matrix4d& T_SUBMAP_BASELINK = pose_stamped.pose;
      Eigen::Matrix4d T_WORLD_BASELINK = T_WORLD_SUBMAP * T_SUBMAP_BASELINK;
      poses_initial.AddSingleTimeStamp(pose_stamped.stamp);
      poses_initial.AddSinglePose(T_WORLD_BASELINK);
    }
  }

  // save
  std::string output_file_initial =
      beam::CombinePaths(output_path, "global_map_trajectory_initial.json");
  poses_initial.WriteToJSON(output_file_initial);
}

void GlobalMap::SaveTrajectoryClouds(const std::string& output_path,
                                     bool save_initial) {
  if (!std::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path, not saving trajectory clouds. Input: {}",
               output_path);
    return;
  }

  // get trajectory
  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  for (uint16_t i = 0; i < submaps_.size(); i++) {
    const SubmapPtr& submap = submaps_.at(i);
    Eigen::Matrix4d T_WORLD_SUBMAP = submap->T_WORLD_SUBMAP();
    std::vector<Submap::PoseStamped> poses_stamped = submap->GetTrajectory();
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

  std::string output_file =
      beam::CombinePaths(output_path, "global_map_trajectory_optimized.pcd");
  BEAM_INFO("Saving trajectory cloud to: {}", output_file);
  std::string error_message;
  if (!beam::SavePointCloud<pcl::PointXYZRGBL>(
          output_file, cloud, beam::PointCloudFileType::PCDBINARY,
          error_message)) {
    BEAM_ERROR("Unable to save trajectory cloud. Reason: {}", error_message);
  }

  if (!save_initial) { return; }

  // get trajectory initial
  pcl::PointCloud<pcl::PointXYZRGBL> cloud_initial;
  for (uint16_t i = 0; i < submaps_.size(); i++) {
    const SubmapPtr& submap = submaps_.at(i);
    Eigen::Matrix4d T_WORLD_SUBMAP = submap->T_WORLD_SUBMAP_INIT();
    auto poses_stamped = submap->GetTrajectory();
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

  if (!beam::SavePointCloud<pcl::PointXYZRGBL>(
          beam::CombinePaths(output_path, "global_map_trajectory_initial.pcd"),
          cloud_initial, beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save trajectory cloud. Reason: {}", error_message);
  }
}

void GlobalMap::SaveSubmapFrames(const std::string& output_path,
                                 bool save_initial) {
  if (!std::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path, not saving submap frames. Input: {}",
               output_path);
    return;
  }

  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  for (uint16_t i = 0; i < submaps_.size(); i++) {
    const SubmapPtr& submap = submaps_.at(i);
    pcl::PointCloud<pcl::PointXYZRGBL> frame =
        beam::CreateFrameCol(submap->Stamp());
    pcl::PointCloud<pcl::PointXYZRGBL> frame_transformed;
    pcl::transformPointCloud(frame, frame_transformed,
                             submap->T_WORLD_SUBMAP());
    cloud += frame_transformed;
  }
  std::string output_file =
      beam::CombinePaths(output_path, "global_map_submap_poses_optimized.pcd");
  BEAM_INFO("Saving submap frames to: {}", output_file);
  std::string error_message{};
  if (!beam::SavePointCloud<pcl::PointXYZRGBL>(
          output_file, cloud, beam::PointCloudFileType::PCDBINARY,
          error_message)) {
    BEAM_ERROR("Unable to save trajectory cloud. Reason: {}", error_message);
  }

  if (!save_initial) { return; }

  pcl::PointCloud<pcl::PointXYZRGBL> cloud_initial;
  for (uint16_t i = 0; i < submaps_.size(); i++) {
    const SubmapPtr& submap = submaps_.at(i);
    pcl::PointCloud<pcl::PointXYZRGBL> frame =
        beam::CreateFrameCol(submap->Stamp());
    pcl::PointCloud<pcl::PointXYZRGBL> frame_transformed;
    pcl::transformPointCloud(frame, frame_transformed,
                             submap->T_WORLD_SUBMAP_INIT());
    cloud_initial += frame_transformed;
  }
  std::string output_file_initial =
      beam::CombinePaths(output_path, "global_map_submap_poses_initial.pcd");
  BEAM_INFO("Saving submap frames to: {}", output_file_initial);
  if (!beam::SavePointCloud<pcl::PointXYZRGBL>(
          output_file_initial, cloud_initial,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save submap frames cloud. Reason: {}", error_message);
  }
}

void GlobalMap::AddRosSubmap(int submap_id) {
  const auto& submap_ptr = submaps_.at(submap_id);

  // get all lidar points in pcl pointcloud
  PointCloud new_submap_pcl_cloud =
      submap_ptr->GetLidarPointsInWorldFrameCombined(true);

  sensor_msgs::PointCloud2 pointcloud2_msg;

  if (new_submap_pcl_cloud.size() > 0) {
    // filter cloud
    new_submap_pcl_cloud = beam_filtering::FilterPointCloud(
        new_submap_pcl_cloud, params_.ros_submap_filter_params);

    // convert to PointCloud2
    pointcloud2_msg = beam::PCLToROS<pcl::PointXYZ>(
        new_submap_pcl_cloud, submap_ptr->Stamp(),
        extrinsics_->GetWorldFrameId(), submap_id + 1);

    // set cloud
    RosMap new_ros_map_lidar(RosMapType::LIDARSUBMAP, pointcloud2_msg);
    ros_submaps_.push(std::make_shared<RosMap>(new_ros_map_lidar));
  }

  // get all camera keypoints in pcl pointcloud
  new_submap_pcl_cloud = submap_ptr->GetKeypointsInWorldFrame(true);

  if (new_submap_pcl_cloud.size() > 0) {
    // convert to PointCloud2
    pointcloud2_msg = beam::PCLToROS<pcl::PointXYZ>(
        new_submap_pcl_cloud, submap_ptr->Stamp(),
        extrinsics_->GetWorldFrameId(), submap_id + 1);

    // set cloud
    RosMap new_ros_map_keypoints(RosMapType::VISUALSUBMAP, pointcloud2_msg);
    ros_submaps_.push(std::make_shared<RosMap>(new_ros_map_keypoints));
  }

  // clear submaps if there are too many in the queue;
  while (ros_submaps_.size() > max_num_ros_submaps_) { ros_submaps_.pop(); }
}

void GlobalMap::AddRosGlobalMap() {
  PointCloud global_lidar_map;
  PointCloud global_keypoints_map;

  for (const auto& submap_ptr : submaps_) {
    // get all lidar points in pcl pointcloud
    PointCloud new_submap_pcl_cloud;
    std::vector<PointCloud> new_submap_points =
        submap_ptr->GetLidarPointsInWorldFrame(10e6, false);
    for (const PointCloud& cloud : new_submap_points) {
      new_submap_pcl_cloud += cloud;
    }

    // filter submap
    new_submap_pcl_cloud = beam_filtering::FilterPointCloud(
        new_submap_pcl_cloud, params_.ros_submap_filter_params);

    // add to global
    global_lidar_map += new_submap_pcl_cloud;

    // get all keypoints in pcl pointcloud
    new_submap_pcl_cloud = submap_ptr->GetKeypointsInWorldFrame(false);

    // add to global
    global_keypoints_map += new_submap_pcl_cloud;
  }

  if (global_lidar_map.size() > 0) {
    // filter global map
    global_lidar_map = beam_filtering::FilterPointCloud(
        global_lidar_map, params_.ros_globalmap_filter_params);

    // convert lidar map to PointCloud2
    sensor_msgs::PointCloud2 pointcloud2_msg = beam::PCLToROS<pcl::PointXYZ>(
        global_lidar_map, last_update_time_, extrinsics_->GetWorldFrameId(),
        global_map_updates_);

    // set cloud
    RosMap new_ros_map_lidar(RosMapType::LIDARGLOBALMAP, pointcloud2_msg);
    ros_global_lidar_map_ = std::make_shared<RosMap>(new_ros_map_lidar);
  }

  if (global_keypoints_map.size() > 0) {
    // convert lidar map to PointCloud2
    sensor_msgs::PointCloud2 pointcloud2_msg = beam::PCLToROS<pcl::PointXYZ>(
        global_keypoints_map, last_update_time_, extrinsics_->GetWorldFrameId(),
        global_map_updates_);

    // set cloud
    RosMap new_ros_map_keypoints(RosMapType::VISUALGLOBALMAP, pointcloud2_msg);
    ros_global_keypoints_map_ = std::make_shared<RosMap>(new_ros_map_keypoints);
  }
}

void GlobalMap::AddNewRosScan(const PointCloud& cloud,
                              const Eigen::Matrix4d& T_WORLD_BASELINK,
                              const ros::Time& stamp) {
  Eigen::Matrix4d T_BASELINK_LIDAR;
  if (!extrinsics_->GetT_BASELINK_LIDAR(T_BASELINK_LIDAR)) {
    BEAM_ERROR("Cannot get extrinsics, not publishing new lidar scans");
    store_new_scans_ = false;
    return;
  }

  PointCloud cloud_in_world_frame;
  Eigen::Matrix4d T_WORLD_LIDAR = T_WORLD_BASELINK * T_BASELINK_LIDAR;
  pcl::transformPointCloud(cloud, cloud_in_world_frame, T_WORLD_LIDAR);

  // convert to PointCloud2
  sensor_msgs::PointCloud2 pointcloud2_msg = beam::PCLToROS<pcl::PointXYZ>(
      cloud_in_world_frame, stamp, extrinsics_->GetWorldFrameId(),
      new_scans_counter_);
  new_scans_counter_++;

  // set cloud
  RosMap new_ros_map(RosMapType::LIDARNEW, pointcloud2_msg);
  ros_new_scans_.push(std::make_shared<RosMap>(new_ros_map));

  // clear maps if there are too many in the queue;
  while (ros_new_scans_.size() > max_num_new_scans_) { ros_new_scans_.pop(); }
}

}} // namespace bs_models::global_mapping
