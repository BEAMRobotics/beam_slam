#include <bs_models/global_mapping/submap.h>

#include <nlohmann/json.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/descriptors/Descriptor.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_utils/filesystem.h>

#include <bs_common/conversions.h>

namespace bs_models { namespace global_mapping {

Submap::Submap(
    const ros::Time& stamp, const Eigen::Matrix4d& T_WORLD_SUBMAP,
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
    const std::shared_ptr<bs_common::ExtrinsicsLookupBase>& extrinsics)
    : stamp_(stamp), camera_model_(camera_model), extrinsics_(extrinsics) {
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
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
    const std::shared_ptr<bs_common::ExtrinsicsLookupBase>& extrinsics)
    : position_(position),
      orientation_(orientation),
      stamp_(stamp),
      camera_model_(camera_model),
      extrinsics_(extrinsics) {
  // convert to eigen transform
  Eigen::Matrix4d T_WORLD_SUBMAP;
  bs_common::FusePoseToEigenTransform(position_, orientation_, T_WORLD_SUBMAP);

  // store initial transforms
  T_WORLD_SUBMAP_ = T_WORLD_SUBMAP;
  T_WORLD_SUBMAP_initial_ = T_WORLD_SUBMAP;
  T_SUBMAP_WORLD_initial_ = beam::InvertTransform(T_WORLD_SUBMAP);
}

fuse_variables::Position3DStamped Submap::Position() const {
  return position_;
}

fuse_variables::Orientation3DStamped Submap::Orientation() const {
  return orientation_;
}

Eigen::Matrix4d Submap::T_WORLD_SUBMAP() const {
  return T_WORLD_SUBMAP_;
}

Eigen::Matrix4d Submap::T_WORLD_SUBMAP_INIT() const {
  return T_WORLD_SUBMAP_initial_;
}

int Submap::Updates() const {
  return graph_updates_;
}

ros::Time Submap::Stamp() const {
  return stamp_;
}

std::string Submap::DescriptorType() const {
  return descriptor_type_;
}

const std::map<uint64_t, ScanPose>& Submap::LidarKeyframes() const {
  return lidar_keyframe_poses_;
}

std::map<uint64_t, ScanPose>::iterator Submap::LidarKeyframesBegin() {
  return lidar_keyframe_poses_.begin();
}

std::map<uint64_t, ScanPose>::iterator Submap::LidarKeyframesEnd() {
  return lidar_keyframe_poses_.end();
}

std::map<uint64_t, Eigen::Matrix4d>::iterator Submap::CameraKeyframesBegin() {
  return camera_keyframe_poses_.begin();
}

std::map<uint64_t, Eigen::Matrix4d>::iterator Submap::CameraKeyframesEnd() {
  return camera_keyframe_poses_.end();
}

std::map<uint64_t, std::vector<Submap::PoseStamped>>::iterator
    Submap::SubframesBegin() {
  return subframe_poses_.begin();
}

std::map<uint64_t, std::vector<Submap::PoseStamped>>::iterator
    Submap::SubframesEnd() {
  return subframe_poses_.end();
}

beam_containers::landmark_container_iterator Submap::LandmarksBegin() {
  return landmarks_.begin();
}

beam_containers::landmark_container_iterator Submap::LandmarksEnd() {
  return landmarks_.end();
}

std::vector<cv::Mat> Submap::GetKeyframeVector() {
  std::vector<cv::Mat> image_vector;
  std::transform(keyframe_images_.begin(), keyframe_images_.end(),
                 std::back_inserter(image_vector),
                 [&](const auto& pair) { return pair.second; });
  return image_vector;
}

const std::map<uint64_t, cv::Mat>& Submap::GetKeyframeMap() {
  return keyframe_images_;
}

void Submap::AddCameraMeasurement(
    const bs_common::CameraMeasurementMsg& camera_measurement,
    const Eigen::Matrix4d& T_WORLDLM_BASELINK) {
  const auto stamp = camera_measurement.header.stamp;
  cv::Mat image;
  if (!camera_measurement.image.data.empty()) {
    image = beam_cv::OpenCVConversions::RosImgToMat(camera_measurement.image);
  }
  const auto sensor_id = camera_measurement.sensor_id;
  const auto measurement_id = camera_measurement.header.seq;

  const auto d_type = beam_cv::Descriptor::StringToDescriptorType(
      camera_measurement.descriptor_type);
  if (!d_type.has_value()) {
    BEAM_WARN("Empty descriptor type in camera measurement message!");
    throw std::runtime_error(
        "Empty descriptor type in camera measurement message!");
  } else if (d_type != beam_cv::DescriptorType::ORB) {
    BEAM_WARN("Invalid descriptor type in camera measurement message, only ORB "
              "accepted.");
    throw std::runtime_error("Invalid descriptor type in camera measurement "
                             "message, only ORB accepted.");
  }

  Eigen::Matrix4d T_SUBMAP_BASELINK =
      T_SUBMAP_WORLD_initial_ * T_WORLDLM_BASELINK;

  camera_keyframe_poses_.emplace(stamp.toNSec(), T_SUBMAP_BASELINK);
  keyframe_images_.emplace(stamp.toNSec(), image);

  const auto landmarks = camera_measurement.landmarks;
  std::for_each(landmarks.begin(), landmarks.end(), [&](const auto& lm_msg) {
    cv::Mat descriptor = beam_cv::Descriptor::VectorDescriptorToCvMat(
        {lm_msg.descriptor.data}, descriptor_type_);
    Eigen::Vector2d value(lm_msg.pixel_u, lm_msg.pixel_v);

    beam_containers::LandmarkMeasurement new_landmark{
        .time_point = stamp,
        .sensor_id = static_cast<uint8_t>(sensor_id),
        .landmark_id = static_cast<uint64_t>(lm_msg.landmark_id),
        .image = static_cast<uint64_t>(measurement_id),
        .value = value,
        .descriptor = descriptor};
    landmarks_.Insert(new_landmark);
  });
}

void Submap::AddLidarMeasurement(const PointCloud& cloud,
                                 const Eigen::Matrix4d& T_WORLDLM_BASELINK,
                                 const ros::Time& stamp) {
  Eigen::Matrix4d T_SUBMAP_BASELINK =
      T_SUBMAP_WORLD_initial_ * T_WORLDLM_BASELINK;
  Eigen::Matrix4d T_BASELINK_LIDAR;
  if (!extrinsics_->GetT_BASELINK_LIDAR(T_BASELINK_LIDAR)) {
    BEAM_ERROR(
        "Cannot get extrinsics, not adding lidar measurement to submap.");
    return;
  }

  // Check if stamp already exists (we may be adding partial scans)
  auto iter = lidar_keyframe_poses_.find(stamp.toNSec());
  if (iter != lidar_keyframe_poses_.end()) {
    // Stamp exists: add cloud to the corresponding scan pose
    iter->second.AddPointCloud(cloud, false);
  } else {
    // Stamp does not exist: add new scanpose to map
    ScanPose new_scan_pose(stamp, T_SUBMAP_BASELINK, T_BASELINK_LIDAR);
    new_scan_pose.AddPointCloud(cloud, false);
    lidar_keyframe_poses_.insert(
        std::pair<uint64_t, ScanPose>(stamp.toNSec(), new_scan_pose));
  }
}

void Submap::AddLidarMeasurement(const beam_matching::LoamPointCloud& cloud,
                                 const Eigen::Matrix4d& T_WORLDLM_BASELINK,
                                 const ros::Time& stamp) {
  Eigen::Matrix4d T_SUBMAP_BASELINK =
      T_SUBMAP_WORLD_initial_ * T_WORLDLM_BASELINK;
  Eigen::Matrix4d T_BASELINK_LIDAR;
  if (!extrinsics_->GetT_BASELINK_LIDAR(T_BASELINK_LIDAR)) {
    BEAM_ERROR(
        "Cannot get extrinsics, not adding lidar measurement to submap.");
    return;
  }

  // Check if stamp already exists (we may be adding partial scans)
  auto iter = lidar_keyframe_poses_.find(stamp.toNSec());
  if (iter != lidar_keyframe_poses_.end()) {
    // Stamp exists: add cloud to the corresponding scan pose
    iter->second.AddPointCloud(cloud, false);
  } else {
    // Stamp does not exist: add new scanpose to map
    ScanPose new_scan_pose(stamp, T_SUBMAP_BASELINK, T_BASELINK_LIDAR);
    new_scan_pose.AddPointCloud(cloud, false);
    lidar_keyframe_poses_.insert(
        std::pair<uint64_t, ScanPose>(stamp.toNSec(), new_scan_pose));
  }
}

void Submap::AddTrajectoryMeasurement(
    const std::vector<Eigen::Matrix4d, beam::AlignMat4d>& poses,
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
    bs_common::FusePoseToEigenTransform(position_, orientation_,
                                        T_WORLD_SUBMAP_);
    return true;
  }
  return false;
}

bool Submap::Near(const ros::Time& time, const double tolerance) const {
  return (std::abs(stamp_.toSec() - time.toSec()) <= tolerance);
}

bool Submap::InSubmap(const ros::Time& time) const {
  const auto start = (*camera_keyframe_poses_.begin()).first;
  const auto end = (*camera_keyframe_poses_.rbegin()).first;
  const auto query = time.toNSec();
  if (query >= start && query <= end) { return true; }
  return false;
}

bool Submap::operator<(const Submap& rhs) const {
  return (stamp_ < rhs.stamp_);
}

void Submap::SaveKeypointsMapInWorldFrame(const std::string& filename,
                                          bool use_initials) {
  BEAM_INFO("Saving final keypoints map to: {}", filename);
  PointCloud map = GetKeypointsInWorldFrame(use_initials);
  if (map.empty()) {
    BEAM_WARN("No keypoints in submap, not saving.");
    return;
  }

  std::string error_message{};
  if (!beam::SavePointCloud<pcl::PointXYZ>(
          filename, map, beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }
}

void Submap::SaveLidarMapInWorldFrame(const std::string& filename,
                                      int max_output_map_size,
                                      bool use_initials) const {
  if (filename.find(".pcd") == std::string::npos) {
    BEAM_ERROR(
        "Invalid filename for saving lidar submap. Needs to be a pcd file. "
        "Input: {}",
        filename);
  }

  std::vector<PointCloud> map =
      GetLidarPointsInWorldFrame(max_output_map_size, use_initials);

  if (map.empty()) {
    BEAM_WARN("No regular lidar points in submap, not saving.");
    return;
  }

  for (int i = 0; i < map.size(); i++) {
    const PointCloud& cloud = map.at(i);
    std::string current_filename = filename;
    std::string replace = "_" + std::to_string(i) + ".pcd";
    current_filename.replace(current_filename.find(".pcd"), 4, replace);
    BEAM_INFO("Saving lidar submap of size {} to: {}", cloud.size(),
              current_filename);
    std::string error_message{};
    if (!beam::SavePointCloud<pcl::PointXYZ>(
            current_filename, cloud, beam::PointCloudFileType::PCDBINARY,
            error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }

    BEAM_INFO("Done saving submap.");
  }
}

void Submap::SaveLidarLoamMapInWorldFrame(const std::string& path,
                                          bool use_initials) const {
  BEAM_INFO("Saving final lidar loam map to: {}", path);
  beam_matching::LoamPointCloud map =
      GetLidarLoamPointsInWorldFrame(use_initials);
  if (map.Size() == 0) {
    BEAM_WARN("No loam points in submap, not saving.");
    return;
  }
  map.SaveCombined(path, "map_in_world.pcd");
}

PointCloud Submap::GetKeypointsInWorldFrame(bool use_initials) {
  TriangulateKeypoints();

  PointCloud cloud;
  for (auto it = landmark_positions_.begin(); it != landmark_positions_.end();
       it++) {
    const Eigen::Vector3d& P_SUBMAP = it->second;
    Eigen::Vector4d P_WORLD;
    if (use_initials) {
      P_WORLD = T_WORLD_SUBMAP_initial_ * P_SUBMAP.homogeneous();
    } else {
      P_WORLD = T_WORLD_SUBMAP_ * P_SUBMAP.homogeneous();
    }
    pcl::PointXYZ p(P_WORLD[0], P_WORLD[1], P_WORLD[2]);
    cloud.push_back(p);
  }
  return cloud;
}

std::vector<PointCloud>
    Submap::GetLidarPointsInWorldFrame(int max_output_map_size,
                                       bool use_initials) const {
  std::vector<PointCloud> map;
  PointCloud map_current;
  for (auto it = lidar_keyframe_poses_.begin();
       it != lidar_keyframe_poses_.end(); it++) {
    const PointCloud& cloud_in_lidar_frame = it->second.Cloud();
    Eigen::Matrix4d T_WORLD_LIDAR;
    if (use_initials) {
      const Eigen::Matrix4d& T_SUBMAP_LIDAR =
          it->second.T_REFFRAME_LIDAR_INIT();
      T_WORLD_LIDAR = T_WORLD_SUBMAP_initial_ * T_SUBMAP_LIDAR;
    } else {
      Eigen::Matrix4d T_SUBMAP_LIDAR = it->second.T_REFFRAME_LIDAR();
      T_WORLD_LIDAR = T_WORLD_SUBMAP_ * T_SUBMAP_LIDAR;
    }

    PointCloud cloud_in_world_frame;
    pcl::transformPointCloud(cloud_in_lidar_frame, cloud_in_world_frame,
                             T_WORLD_LIDAR);

    if (map_current.empty()) {
      map_current = cloud_in_world_frame;
    } else if (map_current.size() + cloud_in_world_frame.size() >
               max_output_map_size) {
      map.push_back(map_current);
      map_current = cloud_in_world_frame;
    } else {
      map_current += cloud_in_world_frame;
    }
  }
  map.push_back(map_current);
  return map;
}

PointCloud Submap::GetLidarPointsInWorldFrameCombined(bool use_initials) const {
  PointCloud map;
  for (auto it = lidar_keyframe_poses_.begin();
       it != lidar_keyframe_poses_.end(); it++) {
    const PointCloud& cloud_in_lidar_frame = it->second.Cloud();
    Eigen::Matrix4d T_WORLD_LIDAR;
    if (use_initials) {
      const Eigen::Matrix4d& T_SUBMAP_LIDAR =
          it->second.T_REFFRAME_LIDAR_INIT();
      T_WORLD_LIDAR = T_WORLD_SUBMAP_initial_ * T_SUBMAP_LIDAR;
    } else {
      Eigen::Matrix4d T_SUBMAP_LIDAR = it->second.T_REFFRAME_LIDAR();
      T_WORLD_LIDAR = T_WORLD_SUBMAP_ * T_SUBMAP_LIDAR;
    }

    PointCloud cloud_in_world_frame;
    pcl::transformPointCloud(cloud_in_lidar_frame, cloud_in_world_frame,
                             T_WORLD_LIDAR);
    map += cloud_in_world_frame;
  }

  return map;
}

beam_matching::LoamPointCloud
    Submap::GetLidarLoamPointsInWorldFrame(bool use_initials) const {
  beam_matching::LoamPointCloud map;
  for (auto it = lidar_keyframe_poses_.begin();
       it != lidar_keyframe_poses_.end(); it++) {
    const beam_matching::LoamPointCloud& cloud_in_lidar_frame =
        it->second.LoamCloud();
    const Eigen::Matrix4d& T_SUBMAP_LIDAR = it->second.T_REFFRAME_LIDAR();
    Eigen::Matrix4d T_WORLD_LIDAR;
    if (use_initials) {
      T_WORLD_LIDAR = T_WORLD_SUBMAP_initial_ * T_SUBMAP_LIDAR;
    } else {
      T_WORLD_LIDAR = T_WORLD_SUBMAP_ * T_SUBMAP_LIDAR;
    }
    beam_matching::LoamPointCloud cloud_in_world_frame(cloud_in_lidar_frame,
                                                       T_WORLD_LIDAR);
    map.Merge(cloud_in_world_frame);
  }
  return map;
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
      if (!extrinsics_->GetT_LIDAR_BASELINK(T_LIDAR_BASELINK)) {
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

  std::vector<Submap::PoseStamped> poses_stamped_vec; // {t, T_SUBMAP_FRAME}
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

  // load submap.json
  std::string submap_path = beam::CombinePaths(input_dir, "submap.json");
  nlohmann::json J_submap;
  if (!beam::ReadJson(submap_path, J_submap)) { return false; }

  // parse json
  try {
    // load general data
    stamp_.fromNSec(J_submap["stamp_nsecs"]);
    graph_updates_ = J_submap["graph_updates"];

    // load position data
    position_ = fuse_variables::Position3DStamped(
        stamp_, fuse_core::uuid::from_string(J_submap["device_id"]));
    std::vector<double> position_vector = J_submap["position_xyz"];
    position_.x() = position_vector.at(0);
    position_.y() = position_vector.at(1);
    position_.z() = position_vector.at(2);

    orientation_ = fuse_variables::Orientation3DStamped(
        stamp_, fuse_core::uuid::from_string(J_submap["device_id"]));
    std::vector<double> orientation_vector = J_submap["orientation_xyzw"];
    orientation_.x() = orientation_vector.at(0);
    orientation_.y() = orientation_vector.at(1);
    orientation_.z() = orientation_vector.at(2);
    orientation_.w() = orientation_vector.at(3);

    std::vector<double> T_WORLD_SUBMAP_vec = J_submap["T_WORLD_SUBMAP"];
    T_WORLD_SUBMAP_ = beam::VectorToEigenTransform(T_WORLD_SUBMAP_vec);

    std::vector<double> T_WORLD_SUBMAP_initial_vec =
        J_submap["T_WORLD_SUBMAP_initial"];
    T_WORLD_SUBMAP_initial_ =
        beam::VectorToEigenTransform(T_WORLD_SUBMAP_initial_vec);
    T_SUBMAP_WORLD_initial_ = beam::InvertTransform(T_WORLD_SUBMAP_initial_);
  } catch (...) {
    BEAM_ERROR("Cannot load submap json, invalid data. Input: {}", submap_path);
    return false;
  }

  // load camera model
  std::string camera_model_path =
      beam::CombinePaths(input_dir, "camera_model.json");
  if (override_camera_model_pointer) {
    if (!boost::filesystem::exists(camera_model_path)) {
      BEAM_ERROR(
          "Cannot load camera model, camera_model.json does not exist inside "
          "root folder. Not loading submap data. Input root directory: {}",
          input_dir);
      return false;
    }
    try {
      camera_model_ = beam_calibration::CameraModel::Create(camera_model_path);
    } catch (...) {
      BEAM_ERROR("Cannot load camera model json at: {}", camera_model_path);
      return false;
    }
  }

  // load camera keyframes json
  nlohmann::json J_cam_keyframes;
  beam::JsonReadErrorType error_type;
  std::string camera_keyframes_path =
      beam::CombinePaths(input_dir, "camera_keyframes.json");
  if (!beam::ReadJson(camera_keyframes_path, J_cam_keyframes, error_type,
                      false)) {
    if (error_type != beam::JsonReadErrorType::EMPTY) {
      BEAM_ERROR(
          "Cannot load camera keyframes json because it is either missing or "
          "has "
          "an invalid extension. Input: {} ",
          camera_keyframes_path);
      return false;
    }
  } else {
    // parse json
    try {
      std::map<uint64_t, std::vector<double>> cam_keyframes_map =
          J_cam_keyframes;
      for (auto it = cam_keyframes_map.begin(); it != cam_keyframes_map.end();
           it++) {
        Eigen::Matrix4d T = beam::VectorToEigenTransform(it->second);
        camera_keyframe_poses_.emplace(it->first, T);
      }
    } catch (...) {
      BEAM_ERROR("Cannot load camera keyframes, invalid data. Input: {}",
                 camera_keyframes_path);
      return false;
    }
  }

  // load landmarks
  std::string landmarks_path = beam::CombinePaths(input_dir, "landmarks.json");
  try {
    landmarks_.LoadFromJson(landmarks_path, false);
  } catch (...) {
    BEAM_ERROR("Cannot load landmarks json, invalid data. Input: {}",
               landmarks_path);
    return false;
  }

  // load lidar keyframes
  std::string lidar_keyframes_root =
      beam::CombinePaths(input_dir, "lidar_keyframes");
  if (!boost::filesystem::exists(lidar_keyframes_root)) {
    BEAM_ERROR(
        "Lidar keyframes folder not found in input root directory, not loading "
        "submap. Input: {}",
        input_dir);
    return false;
  }
  int lidar_keyframe_num = 0;
  while (true) {
    std::string lidar_keyframe_dir = beam::CombinePaths(
        lidar_keyframes_root, "keyframe" + std::to_string(lidar_keyframe_num));
    if (!boost::filesystem::exists(lidar_keyframe_dir)) { break; }
    ScanPose scan_pose(ros::Time(0), Eigen::Matrix4d::Identity());
    try {
      scan_pose.LoadData(lidar_keyframe_dir);
    } catch (...) {
      BEAM_ERROR("Cannot load scanpose data from: {}", lidar_keyframe_dir);
      return false;
    }
    lidar_keyframe_poses_.emplace(scan_pose.Stamp().toNSec(), scan_pose);
    lidar_keyframe_num++;
  }

  // load subframe poses
  std::string subframes_root = beam::CombinePaths(input_dir, "subframes");
  if (!boost::filesystem::exists(subframes_root)) {
    BEAM_ERROR(
        "Subframes folder not found in input root directory, not loading "
        "submap. Input: {}",
        input_dir);
    return false;
  }
  int subframe_num = 0;
  while (true) {
    // check if subframe exists
    std::string subframe_filename = beam::CombinePaths(
        subframes_root, "subframe" + std::to_string(subframe_num));
    if (!boost::filesystem::exists(subframe_filename)) { break; }

    // load subframe json
    nlohmann::json J_subframe;
    beam::JsonReadErrorType error_type;
    if (!beam::ReadJson(subframe_filename, J_subframe, error_type, false)) {
      // we will allow empty files, but skip the rest of the loading
      if (error_type != beam::JsonReadErrorType::EMPTY) {
        BEAM_ERROR(
            "Cannot load subframes json because it is either missing or has "
            "an invalid extension. Input: {} ",
            subframe_filename);
        return false;
      }
      continue;
    }

    std::map<std::string, std::vector<double>> subframe_poses_map;
    uint64_t subframe_stamp;
    try {
      subframe_stamp = J_subframe["subframe_stamp_nsecs"];
      std::map<std::string, std::vector<double>> tmp = J_subframe["poses"];
      subframe_poses_map = tmp;
    } catch (...) {
      BEAM_ERROR("Cannot load subframes json, invalid data. Input: {}",
                 subframe_filename);
      return false;
    }

    std::vector<PoseStamped> subframe_poses_vec;
    for (auto it = subframe_poses_map.begin(); it != subframe_poses_map.end();
         it++) {
      // convert string stamp to integer
      std::istringstream stamp_ss(it->first);
      uint64_t stamp_int;
      stamp_ss >> stamp_int;

      // add subframe pose stamped
      PoseStamped pose_stamped;
      pose_stamped.stamp.fromNSec(stamp_int);
      pose_stamped.pose = beam::VectorToEigenTransform(it->second);
      subframe_poses_vec.push_back(pose_stamped);
    }

    // add to subframes map
    subframe_poses_.emplace(subframe_stamp, subframe_poses_vec);
    subframe_num++;
  }
  return true;
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
      {"num_lidar_keyframes", lidar_keyframe_poses_.size()},
      {"num_camera_keyframes", camera_keyframe_poses_.size()},
      {"num_subframes", subframe_poses_.size()},
      {"num_landmarks", landmarks_.size()},
      {"device_id", fuse_core::uuid::to_string(position_.uuid())},
      {"position_xyz", {position_.x(), position_.y(), position_.z()}},
      {"orientation_xyzw",
       {orientation_.x(), orientation_.y(), orientation_.z(),
        orientation_.w()}}};
  beam::AddTransformToJson(J_submap, T_WORLD_SUBMAP_, "T_WORLD_SUBMAP");
  beam::AddTransformToJson(J_submap, T_WORLD_SUBMAP_initial_,
                           "T_WORLD_SUBMAP_initial");

  std::string submap_filename = beam::CombinePaths(output_dir, "submap.json");
  std::ofstream submap_file(submap_filename);
  submap_file << std::setw(4) << J_submap << std::endl;

  // Save intrinsics
  std::string camera_model_filename =
      beam::CombinePaths(output_dir, "camera_model.json");
  camera_model_->WriteJSON(camera_model_filename);

  // save landmarks
  landmarks_.SaveToJson(beam::CombinePaths(output_dir, "landmarks.json"));

  // save lidar keyframes
  std::string lidar_keyframes_dir =
      beam::CombinePaths(output_dir, "lidar_keyframes");
  boost::filesystem::create_directory(lidar_keyframes_dir);
  int lidar_keyframes_counter = 0;
  for (auto it = lidar_keyframe_poses_.begin();
       it != lidar_keyframe_poses_.end(); it++) {
    // create new directory
    std::string keyframe_dir = beam::CombinePaths(
        lidar_keyframes_dir,
        "keyframe" + std::to_string(lidar_keyframes_counter));
    boost::filesystem::create_directory(keyframe_dir);

    // call save on keyframe
    it->second.SaveData(keyframe_dir);
    lidar_keyframes_counter++;
  }

  // save camera keyframes
  nlohmann::json J_camera_keyframes;
  for (auto it = camera_keyframe_poses_.begin();
       it != camera_keyframe_poses_.end(); it++) {
    beam::AddPoseToJson(J_camera_keyframes, it->first, it->second);
  }
  std::string camera_keyframes_filename =
      beam::CombinePaths(output_dir, "camera_keyframes.json");
  std::ofstream camera_keyframe_file(camera_keyframes_filename);
  camera_keyframe_file << std::setw(4) << J_camera_keyframes << std::endl;

  // save keyframe images
  std::string keyframe_dir = beam::CombinePaths(output_dir, "image_keyframes");
  for (const auto& [time, image] : keyframe_images_) {
    std::string keyframe_filename =
        beam::CombinePaths(keyframe_dir, std::to_string(time) + ".png");
    cv::imwrite(keyframe_filename, image);
  }

  // save subframes
  std::string subframe_dir = beam::CombinePaths(output_dir, "subframes");
  boost::filesystem::create_directory(subframe_dir);
  int subframes_counter = 0;
  for (auto it = subframe_poses_.begin(); it != subframe_poses_.end(); it++) {
    // create json
    nlohmann::json J_subframes;
    J_subframes["subframe_stamp_nsecs"] = it->first;

    // add poses
    nlohmann::json J_subframes_poses;
    for (const auto& pose_stamped : it->second) {
      beam::AddPoseToJson(J_subframes_poses, pose_stamped.stamp.toNSec(),
                          pose_stamped.pose);
    }
    J_subframes["poses"] = J_subframes_poses;

    // save to file
    std::string subframe_filename = beam::CombinePaths(
        subframe_dir, "subframe" + std::to_string(subframes_counter) + ".json");
    std::ofstream subframe_file(subframe_filename);
    subframe_file << std::setw(4) << J_subframes << std::endl;
    subframes_counter++;
  }
}

void Submap::TriangulateKeypoints(bool override_points) {
  if (landmark_positions_.size() != 0 && !override_points) { return; }

  // get the sensor id for the first landmark measurement.
  // Note: this will only work for one camera
  auto sensor_id = landmarks_.begin()->sensor_id;

  // get all landmark ids
  std::vector<uint64_t> landmark_ids = landmarks_.GetLandmarkIDs();

  // iterate through all landmarks and triangulate point based on track
  for (auto landmark_id : landmark_ids) {
    // get track
    auto track = landmarks_.GetTrack(landmark_id);
    if (track.size() < 2) { continue; }

    // precompute inverse submap pose
    Eigen::Matrix4d T_SUBMAP_WORLD = beam::InvertTransform(T_WORLD_SUBMAP_);

    // get poses and pixels for each measurement in the track
    std::vector<Eigen::Matrix4d, beam::AlignMat4d> Ts_CAM_WORLD;
    std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
    for (const beam_containers::LandmarkMeasurement& measurement : track) {
      // find keyframe from measurement stamp
      std::map<uint64_t, Eigen::Matrix4d>::const_iterator keyframe_pose =
          camera_keyframe_poses_.find(measurement.time_point.toNSec());
      if (keyframe_pose == camera_keyframe_poses_.end()) { continue; }

      // get transform from world to camera to project points
      const Eigen::Matrix4d& T_SUBMAP_BASELINK = keyframe_pose->second;
      Eigen::Matrix4d T_BASELINK_SUBMAP =
          beam::InvertTransform(T_SUBMAP_BASELINK);
      Eigen::Matrix4d T_CAM_BASELINK(Eigen::Matrix4d::Identity());
      if (!extrinsics_->GetT_CAMERA_IMU(T_CAM_BASELINK)) {
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
    auto point = beam_cv::Triangulation::TriangulatePoint(
        camera_model_, Ts_CAM_WORLD, pixels, 100.0, 20.0);
    landmark_positions_.emplace(landmark_id, point.value());
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
    BEAM_ERROR("Trajectory measurement stamp does not match with any lidar or "
               "camera keyframe.");
    return false;
  }
}

}} // namespace bs_models::global_mapping
