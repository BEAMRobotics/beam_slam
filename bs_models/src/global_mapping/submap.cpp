#include <bs_models/global_mapping/submap.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <bs_common/utils.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_cv/descriptors/Descriptor.h>

namespace bs_models {

namespace global_mapping {

nlohmann::json TransformToJson(const Eigen::Matrix4d& T,
                               const std::string& name) {
  nlohmann::json J = {{name,
                       {T(0, 0), T(0, 1), T(0, 2), T(0, 3), T(1, 0), T(1, 1),
                        T(1, 2), T(1, 3), T(2, 0), T(2, 1), T(2, 2), T(2, 3),
                        T(3, 0), T(3, 1), T(3, 2), T(3, 3)}}};
  return J;
}

void AddTransformToJson(nlohmann::json& J, const Eigen::Matrix4d& T,
                        const std::string& name) {
  J.update(TransformToJson(T, name));
}

nlohmann::json ToJsonPoseObject(uint64_t t, const Eigen::Matrix4d& T) {
  nlohmann::json pose_object = {
      {"nsecs", t},
      {"T",
       {T(0, 0), T(0, 1), T(0, 2), T(0, 3), T(1, 0), T(1, 1), T(1, 2), T(1, 3),
        T(2, 0), T(2, 1), T(2, 2), T(2, 3), T(3, 0), T(3, 1), T(3, 2),
        T(3, 3)}}};
  return pose_object;
}

Submap::Submap(
    const ros::Time& stamp, const Eigen::Matrix4d& T_WORLD_SUBMAP,
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model)
    : stamp_(stamp), camera_model_(camera_model) {
  // create fuse variables
  position_ = fuse_variables::Position3DStamped(stamp, fuse_core::uuid::NIL);
  orientation_ =
      fuse_variables::Orientation3DStamped(stamp, fuse_core::uuid::NIL);

  // add transform
  bs_common::EigenTransformToFusePose(T_WORLD_SUBMAP, position_, orientation_);

  // store initial transforms
  T_WORLD_SUBMAP_ = T_WORLD_SUBMAP;
  T_WORLD_SUBMAP_initial_ = T_WORLD_SUBMAP;
  T_SUBMAP_WORLD_initial_ = beam::InvertTransform(T_WORLD_SUBMAP);
}

Submap::Submap(
    const ros::Time& stamp, const fuse_variables::Position3DStamped& position,
    const fuse_variables::Orientation3DStamped& orientation,
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model)
    : position_(position),
      orientation_(orientation),
      stamp_(stamp),
      camera_model_(camera_model) {
  // convert to eigen transform
  Eigen::Matrix4d T_WORLD_SUBMAP;
  FusePoseToEigenTransform(position_, orientation_, T_WORLD_SUBMAP);

  // store initial transforms
  T_WORLD_SUBMAP_ = T_WORLD_SUBMAP;
  T_WORLD_SUBMAP_initial_ = T_WORLD_SUBMAP;
  T_SUBMAP_WORLD_initial_ = beam::InvertTransform(T_WORLD_SUBMAP);
}

fuse_variables::Position3DStamped Submap::Position() const { return position_; }

fuse_variables::Orientation3DStamped Submap::Orientation() const {
  return orientation_;
}

Eigen::Matrix4d Submap::T_WORLD_SUBMAP() const { return T_WORLD_SUBMAP_; }

Eigen::Matrix4d Submap::T_WORLD_SUBMAP_INIT() const {
  return T_WORLD_SUBMAP_initial_;
}

int Submap::Updates() const { return graph_updates_; }

ros::Time Submap::Stamp() const { return stamp_; }

void Submap::AddCameraMeasurement(
    const std::vector<LandmarkMeasurementMsg>& landmarks,
    uint8_t descriptor_type_int, const Eigen::Matrix4d& T_WORLDLM_BASELINK,
    const ros::Time& stamp, int sensor_id, int measurement_id) {
  Eigen::Matrix4d T_SUBMAP_BASELINK =
      T_SUBMAP_WORLD_initial_ * T_WORLDLM_BASELINK;

  camera_keyframe_poses_.emplace(stamp.toNSec(), T_SUBMAP_BASELINK);

  for (const LandmarkMeasurementMsg& landmark_msg : landmarks) {
    Eigen::Vector2d value(landmark_msg.pixel_u, landmark_msg.pixel_v);
    auto descriptor_type =
        beam_cv::DescriptorTypeIntMap.find(descriptor_type_int);
    if (descriptor_type == beam_cv::DescriptorTypeIntMap.end()) {
      BEAM_ERROR(
          "Invalid descriptor type in LandMarkMeasurementMsg. Skipping "
          "measurement.");
      continue;
    }
    cv::Mat descriptor = beam_cv::Descriptor::CreateDescriptor(
        landmark_msg.descriptor, descriptor_type->second);
    beam_containers::LandmarkMeasurement new_landmark{
        .time_point = stamp,
        .sensor_id = static_cast<uint8_t>(sensor_id),
        .landmark_id = static_cast<uint64_t>(landmark_msg.landmark_id),
        .image = static_cast<uint64_t>(measurement_id),
        .value = value,
        .descriptor = descriptor};
    landmarks_.Insert(new_landmark);
  }
}

void Submap::AddLidarMeasurement(const PointCloud& cloud,
                                 const Eigen::Matrix4d& T_WORLDLM_BASELINK,
                                 const ros::Time& stamp, int type) {
  Eigen::Matrix4d T_SUBMAP_BASELINK =
      T_SUBMAP_WORLD_initial_ * T_WORLDLM_BASELINK;
  Eigen::Matrix4d T_BASELINK_LIDAR;
  if (!extrinsics_.GetT_BASELINK_LIDAR(T_BASELINK_LIDAR, stamp)) {
    BEAM_ERROR(
        "Cannot get extrinsics, not adding lidar measurement to submap.");
    return;
  }
  Eigen::Matrix4d T_SUBMAP_LIDAR = T_SUBMAP_BASELINK * T_BASELINK_LIDAR;

  // Check if stamp already exists (we may be adding partial scans)
  auto iter = lidar_keyframe_poses_.find(stamp.toNSec());
  if (iter != lidar_keyframe_poses_.end()) {
    // Stamp exists: add cloud to the corresponding scan pose
    iter->second.AddPointCloud(cloud, type, false);
  } else {
    // Stamp does not exist: add new scanpose to map
    bs_common::ScanPose new_scan_pose(stamp, T_SUBMAP_LIDAR, T_BASELINK_LIDAR);
    new_scan_pose.AddPointCloud(cloud, type, false);
    lidar_keyframe_poses_.insert(std::pair<uint64_t, bs_common::ScanPose>(
        stamp.toNSec(), new_scan_pose));
  }
}

void Submap::AddTrajectoryMeasurement(
    const std::vector<Eigen::Matrix4d, pose_allocator>& poses,
    const std::vector<ros::Time>& stamps, const ros::Time& stamp) {
  std::vector<PoseStamped> poses_stamped;
  for (int i = 0; i < poses.size(); i++) {
    PoseStamped pose_stamped{.stamp = stamps[i], .pose = poses[i]};
    poses_stamped.push_back(pose_stamped);
  }

  auto iter = subframe_poses_.find(stamp.toNSec());
  if (iter != subframe_poses_.end()) {
    BEAM_WARN("Overriding trajectory measurement w.r.t. keyframe at time: {}",
              stamp.toSec());
    iter->second = poses_stamped;
  } else {
    subframe_poses_.emplace(stamp.toNSec(), poses_stamped);
  }
}

bool Submap::UpdatePose(fuse_core::Graph::ConstSharedPtr graph_msg) {
  if (graph_msg->variableExists(position_.uuid()) &&
      graph_msg->variableExists(orientation_.uuid())) {
    position_ = dynamic_cast<const fuse_variables::Position3DStamped&>(
        graph_msg->getVariable(position_.uuid()));

    orientation_ = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
        graph_msg->getVariable(orientation_.uuid()));
    graph_updates_++;
    FusePoseToEigenTransform(position_, orientation_, T_WORLD_SUBMAP_);
    return true;
  }
  return false;
}

bool Submap::Near(const ros::Time& time, const double tolerance) const {
  return (std::abs(stamp_.toSec() - time.toSec()) <= tolerance);
}

bool Submap::operator<(const Submap& rhs) const {
  return (stamp_ < rhs.stamp_);
}

void Submap::SaveKeypointsMapInWorldFrame(const std::string& filename,
                                          bool use_initial_world_frame) {
  BEAM_INFO("Saving final keypoints map to: {}", filename);
  PointCloud map = GetKeypointsInWorldFrame(use_initial_world_frame);
  if (map.empty()) {
    BEAM_WARN("No keypoints in submap, not saving.");
    return;
  }
  pcl::io::savePCDFileASCII(filename, map);
}

void Submap::SaveLidarMapInWorldFrame(const std::string& filename,
                                      int max_output_map_size,
                                      bool use_initial_world_frame) const {
  if (filename.find(".pcd") == std::string::npos) {
    BEAM_ERROR(
        "Invalid filename for saving lidar submap. Needs to be a pcd file. "
        "Input: {}",
        filename);
  }

  std::vector<PointCloud> map =
      GetLidarPointsInWorldFrame(max_output_map_size, use_initial_world_frame);
  if (map.empty()) {
    BEAM_WARN("No regular lidar points in submap, not saving.");
    return;
  }

  for (int i = 0; i < map.size(); i++) {
    const PointCloud& cloud = map[i];
    std::string current_filename = filename;
    std::string replace = "_" + std::to_string(i) + ".pcd";
    current_filename.replace(current_filename.find(".pcd"), 4, replace);
    BEAM_INFO("Saving lidar submap to: {}", current_filename);
    pcl::io::savePCDFileASCII(current_filename, cloud);
  }
}

void Submap::SaveLidarLoamMapInWorldFrame(const std::string& path,
                                          bool combine_features,
                                          bool use_initial_world_frame) const {
  BEAM_INFO("Saving final lidar loam map to: {}", path);
  beam_matching::LoamPointCloud map =
      GetLidarLoamPointsInWorldFrame(use_initial_world_frame);
  if (map.Size() == 0) {
    BEAM_WARN("No loam points in submap, not saving.");
    return;
  }
  map.Save(path, combine_features);
}

PointCloud Submap::GetKeypointsInWorldFrame(bool use_initial_world_frame) {
  TriangulateKeypoints();

  PointCloud cloud;
  for (auto it = landmark_positions_.begin(); it != landmark_positions_.end();
       it++) {
    const Eigen::Vector3d& P_SUBMAP = it->second;
    Eigen::Vector4d P_WORLD;
    if (use_initial_world_frame) {
      P_WORLD = T_WORLD_SUBMAP_initial_ * P_SUBMAP.homogeneous();
    } else {
      P_WORLD = T_WORLD_SUBMAP_ * P_SUBMAP.homogeneous();
    }
    pcl::PointXYZ p(P_WORLD[0], P_WORLD[1], P_WORLD[2]);
    cloud.push_back(p);
  }
  return cloud;
}

std::vector<PointCloud> Submap::GetLidarPointsInWorldFrame(
    int max_output_map_size, bool use_initial_world_frame) const {
  std::vector<PointCloud> map;
  PointCloud map_current;
  for (auto it = lidar_keyframe_poses_.begin();
       it != lidar_keyframe_poses_.end(); it++) {
    const PointCloud& cloud_scanframe = it->second.Cloud();
    const Eigen::Matrix4d& T_SUBMAP_SCAN = it->second.T_REFFRAME_LIDAR();
    Eigen::Matrix4d T_WORLD_SCAN;
    if (use_initial_world_frame) {
      T_WORLD_SCAN = T_WORLD_SUBMAP_initial_ * T_SUBMAP_SCAN;
    } else {
      T_WORLD_SCAN = T_WORLD_SUBMAP_ * T_SUBMAP_SCAN;
    }

    PointCloud cloud_worldframe;
    pcl::transformPointCloud(cloud_scanframe, cloud_worldframe, T_WORLD_SCAN);
    if (map_current.empty()) {
      map_current = cloud_worldframe;
    } else if (map_current.size() + cloud_worldframe.size() >
               max_output_map_size) {
      map.push_back(map_current);
      map_current = cloud_worldframe;
    } else {
      map_current += cloud_worldframe;
    }
  }
  map.push_back(map_current);
  return map;
}

beam_matching::LoamPointCloud Submap::GetLidarLoamPointsInWorldFrame(
    bool use_initial_world_frame) const {
  beam_matching::LoamPointCloud map;
  for (auto it = lidar_keyframe_poses_.begin();
       it != lidar_keyframe_poses_.end(); it++) {
    const beam_matching::LoamPointCloud& cloud_scanframe =
        it->second.LoamCloud();
    const Eigen::Matrix4d& T_SUBMAP_SCAN = it->second.T_REFFRAME_LIDAR();
    Eigen::Matrix4d T_WORLD_SCAN;
    if (use_initial_world_frame) {
      T_WORLD_SCAN = T_WORLD_SUBMAP_initial_ * T_SUBMAP_SCAN;
    } else {
      T_WORLD_SCAN = T_WORLD_SUBMAP_ * T_SUBMAP_SCAN;
    }
    beam_matching::LoamPointCloud cloud_worldframe = cloud_scanframe;
    cloud_worldframe.TransformPointCloud(T_WORLD_SCAN);
    map.Merge(cloud_worldframe);
  }
}

std::vector<Submap::PoseStamped> Submap::GetTrajectory() const {
  // first we create an ordered map so we can easily make sure poses are in
  // order, then we'll convert to a vector
  std::map<uint64_t, Eigen::Matrix4d> poses_stamped_map;

  // get all camera keyframe poses
  for (auto it = camera_keyframe_poses_.begin();
       it != camera_keyframe_poses_.end(); it++) {
    poses_stamped_map.emplace(it->first, it->second);
  }

  // get all lidar keyframe poses if they do not override camera poses
  for (auto it = lidar_keyframe_poses_.begin();
       it != lidar_keyframe_poses_.end(); it++) {
    if (poses_stamped_map.find(it->first) == poses_stamped_map.end()) {
      // transform to baselink pose
      const Eigen::Matrix4d& T_SUBMAP_LIDAR = it->second.T_REFFRAME_LIDAR();
      ros::Time stamp;
      stamp.fromNSec(it->first);
      Eigen::Matrix4d T_LIDAR_BASELINK;
      if (!extrinsics_.GetT_LIDAR_BASELINK(T_LIDAR_BASELINK, stamp)) {
        BEAM_ERROR(
            "Cannot get extrinsics, not adding lidar pose to trajectory.");
        continue;
      }

      // add to map
      poses_stamped_map.emplace(it->first, T_SUBMAP_LIDAR * T_LIDAR_BASELINK);
    }
  }

  // get all subframe poses
  for (auto it = subframe_poses_.begin(); it != subframe_poses_.end(); it++) {
    // get keyframe pose
    Eigen::Matrix4d T_SUBMAP_KEYFRAME;
    if (!FindT_SUBMAP_KEYFRAME(it->first, T_SUBMAP_KEYFRAME)) {
      BEAM_ERROR("Not adding trajectory measurement to final trajetory.");
      continue;
    }

    // get poses w.r.t. submap & add to final poses map
    const std::vector<PoseStamped>& relative_poses_stamped = it->second;
    for (const PoseStamped& relative_pose_stamped : relative_poses_stamped) {
      const Eigen::Matrix4d& T_KEYFRAME_FRAME = relative_pose_stamped.pose;
      poses_stamped_map.emplace(relative_pose_stamped.stamp.toNSec(),
                                T_SUBMAP_KEYFRAME * T_KEYFRAME_FRAME);
    }
  }

  std::vector<Submap::PoseStamped> poses_stamped_vec;  // {t, T_SUBMAP_FRAME}
  for (auto it = poses_stamped_map.begin(); it != poses_stamped_map.end();
       it++) {
    ros::Time new_stamp;
    new_stamp.fromNSec(it->first);
    Submap::PoseStamped new_pose{.stamp = new_stamp, .pose = it->second};
    poses_stamped_vec.push_back(new_pose);
  }
  return poses_stamped_vec;
}

void Submap::Print(std::ostream& stream) const {
  // calculate number of subframes
  int num_subframes = 0;
  for (auto it = subframe_poses_.begin(); it != subframe_poses_.end(); it++) {
    num_subframes += it->second.size();
  }

  stream << "  Stamp: " << stamp_ << "\n"
         << "  Number of updates: " << graph_updates_ << "\n"
         << "  Position:\n"
         << "  - x: " << position_.x() << "\n"
         << "  - y: " << position_.y() << "\n"
         << "  - z: " << position_.z() << "\n"
         << "  Orientation:\n"
         << "  - x: " << orientation_.x() << "\n"
         << "  - y: " << orientation_.y() << "\n"
         << "  - z: " << orientation_.z() << "\n"
         << "  - w: " << orientation_.w() << "\n"
         << "  Number of lidar keyframes: " << lidar_keyframe_poses_.size()
         << "\n"
         << "  Number of camera keyframes: " << camera_keyframe_poses_.size()
         << "\n"
         << "  Number of landmarks: " << landmarks_.size() << "\n"
         << "  Number of subframes: " << num_subframes << "\n";
}

bool Submap::LoadData(const std::string& input_dir,
                      bool override_camera_model_pointer) {
  if (!boost::filesystem::exists(input_dir)) {
    BEAM_ERROR("Invalid input directory, not loading submap data. Input: {}",
               input_dir);
    return false;
  }

  // TODO
}

void Submap::SaveData(const std::string& output_dir) {
  if (!boost::filesystem::exists(output_dir)) {
    BEAM_ERROR("Invalid output directory, not saving submap data. Input: {}",
               output_dir);
    return;
  }

  // First, save general submap data to a json
  nlohmann::json J_submap = {
      {"stamp_nsecs", stamp_.toNSec()},
      {"graph_updates", graph_updates_},
      {"position_xyz", {position_.x(), position_.y(), position_.z()}},
      {"orientation_xyzw",
       {orientation_.x(), orientation_.y(), orientation_.z(),
        orientation_.w()}}};
  AddTransformToJson(J_submap, T_WORLD_SUBMAP_, "T_WORLD_SUBMAP");
  AddTransformToJson(J_submap, T_WORLD_SUBMAP_initial_,
                     "T_WORLD_SUBMAP_initial");

  std::string submap_filename = output_dir + "submap.json";
  std::ofstream submap_file(submap_filename);
  submap_file << std::setw(4) << J_submap << std::endl;

  // Save intrinsics
  std::string camera_model_filename = output_dir + "camera_model.json";
  camera_model_->WriteJSON(camera_model_filename);

  // save landmarks
  // TODO

  // save lidar keyframes
  // TODO

  // save camera keyframes
  nlohmann::json J_camera_keyframes_array = nlohmann::json::array();
  for (auto it = camera_keyframe_poses_.begin();
       it != camera_keyframe_poses_.end(); it++) {
    const Eigen::Matrix4d& T = it->second;
    J_camera_keyframes_array.push_back(ToJsonPoseObject(it->first, it->second));
  }
  nlohmann::json J_camera_keyframes;
  J_camera_keyframes["poses"] = J_camera_keyframes_array;
  std::string camera_keyframes_filename = output_dir + "camera_keyframes.json";
  std::ofstream camera_keyframe_file(camera_keyframes_filename);
  camera_keyframe_file << std::setw(4) << J_camera_keyframes << std::endl;

  // save subframes
  // TODO
}

void Submap::TriangulateKeypoints(bool override_points) {
  if (landmark_positions_.size() != 0 && !override_points) {
    return;
  }

  // get the sensor id for the first landmark measurement.
  // Note: this will only work for one camera
  auto sensor_id = landmarks_.begin()->sensor_id;

  // get all landmark ids
  std::vector<uint64_t> landmark_ids =
      landmarks_.GetLandmarkIDsInWindow(ros::TIME_MIN, ros::TIME_MAX);

  // iterate through all landmarks and triangulate point based on track
  for (auto landmark_id : landmark_ids) {
    // get track
    auto track = landmarks_.GetTrackInWindow(sensor_id, landmark_id,
                                             ros::TIME_MIN, ros::TIME_MAX);
    if (track.size() < 2) {
      continue;
    }

    // precompute inverse submap pose
    Eigen::Matrix4d T_SUBMAP_WORLD = beam::InvertTransform(T_WORLD_SUBMAP_);

    // get poses and pixels for each measurement in the track
    std::vector<Eigen::Matrix4d, beam_cv::AlignMat4d> Ts_CAM_WORLD;
    std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
    for (const beam_containers::LandmarkMeasurement& measurement : track) {
      // find keyframe from measurement stamp
      std::map<uint64_t, Eigen::Matrix4d>::const_iterator keyframe_pose =
          camera_keyframe_poses_.find(measurement.time_point.toNSec());
      if (keyframe_pose == camera_keyframe_poses_.end()) {
        continue;
      }

      // get transform from world to camera to project points
      const Eigen::Matrix4d& T_SUBMAP_BASELINK = keyframe_pose->second;
      Eigen::Matrix4d T_BASELINK_SUBMAP =
          beam::InvertTransform(T_SUBMAP_BASELINK);
      Eigen::Matrix4d T_CAM_BASELINK(Eigen::Matrix4d::Identity());
      if (!extrinsics_.GetT_CAMERA_IMU(T_CAM_BASELINK)) {
        BEAM_ERROR(
            "Cannot lookup transform from camera to IMU. Using identity.");
      }
      Eigen::Matrix4d T_CAM_WORLD =
          T_CAM_BASELINK * T_BASELINK_SUBMAP * T_SUBMAP_WORLD;

      // add results to vector
      Ts_CAM_WORLD.push_back(T_CAM_WORLD);
      pixels.push_back(measurement.value.cast<int>());
    }

    // triangulate point and add if successful
    beam::opt<Eigen::Vector3d> point = beam_cv::Triangulation::TriangulatePoint(
        camera_model_, Ts_CAM_WORLD, pixels);
    if (point.has_value()) {
      landmark_positions_.emplace(landmark_id, point.value());
    }
  }
}

bool Submap::FindT_SUBMAP_KEYFRAME(uint64_t time,
                                   Eigen::Matrix4d& T_SUBMAP_KEYFRAME) const {
  // first, look for timestamp in camera keyframe poses (these take priority)
  auto iter_cam = camera_keyframe_poses_.find(time);
  if (iter_cam != camera_keyframe_poses_.end()) {
    T_SUBMAP_KEYFRAME = iter_cam->second;
    return true;
  }

  auto iter_lid = lidar_keyframe_poses_.find(time);
  if (iter_lid != lidar_keyframe_poses_.end()) {
    const Eigen::Matrix4d& T_SUBMAP_LIDAR = iter_lid->second.T_REFFRAME_LIDAR();
    Eigen::Matrix4d T_LIDAR_BASELINK =
        beam::InvertTransform(iter_lid->second.T_BASELINK_LIDAR());
    T_SUBMAP_KEYFRAME = T_SUBMAP_LIDAR * T_LIDAR_BASELINK;
    return true;
  }

  else {
    BEAM_ERROR(
        "Trajectory measurement stamp does not match with any lidar or "
        "camera keyframe.");
    return false;
  }
}

}  // namespace global_mapping

}  // namespace bs_models
