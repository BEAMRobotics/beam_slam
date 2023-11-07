#include <bs_models/scan_registration/registration_map.h>

#include <boost/filesystem.hpp>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <bs_common/conversions.h>
#include <bs_common/extrinsics_lookup_online.h>

namespace bs_models { namespace scan_registration {

using namespace beam_matching;

RegistrationMap::RegistrationMap() {
  bs_common::ExtrinsicsLookupOnline& extrinsics_online =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
  world_frame_id_ = extrinsics_online.GetWorldFrameId();

  // setup publishers
  ros::NodeHandle n;
  lidar_map_publisher_ = n.advertise<sensor_msgs::PointCloud2>(
      "/local_mapper/local_map/lidar_map", 10);
  loam_map_publisher_ = n.advertise<sensor_msgs::PointCloud2>(
      "/local_mapper/local_map/loam_map", 10);
}

RegistrationMap& RegistrationMap::GetInstance() {
  static RegistrationMap instance;
  return instance;
}

bool RegistrationMap::SetParams(int map_size, bool publish_updates) {
  publish_updates_ = publish_updates;

  if (map_params_set_ && map_size != map_size_) {
    BEAM_WARN(
        "Map parameters already set, overriding and purging extra clouds.");
    // in case the map size decreased and existing scans are here, let's purge
    while (scans_.size() > map_size) {
      uint64_t first_scan_stamp = scans_.begin()->first;
      scans_.erase(first_scan_stamp);
    }
  }

  map_size_ = map_size;
  map_params_set_ = true;
  return true;
}

int RegistrationMap::MapSize() const {
  return map_size_;
}

bool RegistrationMap::Empty() const {
  return scans_.empty();
}

int RegistrationMap::NumScans() const {
  return scans_.size();
}

void RegistrationMap::AddPointCloud(const PointCloud& cloud,
                                    const LoamPointCloud& loam_cloud,
                                    const ros::Time& stamp,
                                    const Eigen::Matrix4d& T_Map_Scan) {
  // transform to map
  scans_.emplace(stamp.toNSec(), ScanPoseInMapFrame());
  ScanPoseInMapFrame& scan = scans_.at(stamp.toNSec());
  scan.T_Map_Scan = T_Map_Scan;
  pcl::transformPointCloud(cloud, scan.cloud, T_Map_Scan);
  scan.loam_cloud = LoamPointCloud(loam_cloud, T_Map_Scan);
  scan.orientation_uuid = fuse_core::uuid::generate(
      "fuse_variables::Orientation3DStamped", stamp, fuse_core::uuid::NIL);
  scan.position_uuid = fuse_core::uuid::generate(
      "fuse_variables::Position3DStamped", stamp, fuse_core::uuid::NIL);

  // remove cloud & pose if map is greater than max size
  if (scans_.size() > map_size_) {
    uint64_t first_scan_stamp = scans_.begin()->first;
    scans_.erase(first_scan_stamp);
  }

  Publish();
}

PointCloud RegistrationMap::GetPointCloudMap() const {
  PointCloud cloud;
  for (auto it = scans_.begin(); it != scans_.end(); it++) {
    cloud += it->second.cloud;
  }
  return cloud;
}

LoamPointCloud RegistrationMap::GetLoamCloudMap() const {
  LoamPointCloud cloud;
  for (auto it = scans_.begin(); it != scans_.end(); it++) {
    cloud.Merge(it->second.loam_cloud);
  }
  return cloud;
}

bool RegistrationMap::UpdateScan(const ros::Time& stamp,
                                 const Eigen::Matrix4d& T_Map_Scan,
                                 double rotation_threshold_deg,
                                 double translation_threshold_m) {
  bool scan_found{false};
  uint64_t stamp_nsecs = stamp.toNSec();

  // update regular pointclouds
  auto it = scans_.find(stamp_nsecs);
  if (it == scans_.end()) { return false; }

  auto& scan = it->second;
  // check poses are not too similar
  if (beam::ArePosesEqual(T_Map_Scan, scan.T_Map_Scan, rotation_threshold_deg,
                          translation_threshold_m)) {
    return false;
  }

  // update pointclouds
  Eigen::Matrix4d T_MAPNEW_MAPOLD =
      T_Map_Scan * beam::InvertTransform(scan.T_Map_Scan);
  pcl::transformPointCloud(scan.cloud, scan.cloud, T_MAPNEW_MAPOLD);
  scan.loam_cloud.TransformPointCloud(T_MAPNEW_MAPOLD);
  scan.T_Map_Scan = T_Map_Scan;

  Publish();
  return true;
}

void RegistrationMap::Save(const std::string& save_path, bool add_frames,
                           uint8_t r, uint8_t g, uint8_t b) const {
  if (!boost::filesystem::exists(save_path)) {
    BEAM_ERROR("Invalid output path for RegistrationMap: {}", save_path);
    return;
  }

  if (scans_.empty()) {
    BEAM_ERROR("Registration map is empty, not saving map");
    return;
  }

  LoamPointCloud loam_map = GetLoamCloudMap();
  if (!loam_map.Empty()) {
    loam_map.SaveCombined(save_path, "registration_map_loam.pcd");
  }

  PointCloud map = GetPointCloudMap();
  std::string error_message;
  if (!map.empty()) {
    PointCloudCol map_col = beam::ColorPointCloud(map, r, g, b);
    std::string map_path =
        beam::CombinePaths(save_path, "registration_map.pcd");
    if (!beam::SavePointCloud<pcl::PointXYZRGB>(
            map_path, map_col, beam::PointCloudFileType::PCDBINARY,
            error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
  }

  pcl::PointCloud<pcl::PointXYZRGBL> poses_cloud;
  for (const auto& [stamp_ns, scan] : scans_) {
    ros::Time t;
    t.fromNSec(stamp_ns);
    pcl::PointCloud<pcl::PointXYZRGBL> frame = beam::CreateFrameCol(t);
    beam::MergeFrameToCloud(poses_cloud, frame, scan.T_Map_Scan);
  }

  std::string poses_path =
      beam::CombinePaths(save_path, "registration_map_poses.pcd");
  if (!beam::SavePointCloud<pcl::PointXYZRGBL>(
          poses_path, poses_cloud, beam::PointCloudFileType::PCDBINARY,
          error_message)) {
    BEAM_ERROR("Unable to registration map poses cloud. Reason: {}",
               error_message);
  }
}

bool RegistrationMap::GetScanPose(const ros::Time& stamp,
                                  Eigen::Matrix4d& T_Map_Scan) const {
  auto iter = scans_.find(stamp.toNSec());
  if (iter == scans_.end()) { return false; }

  T_Map_Scan = iter->second.T_Map_Scan;
  return true;
}

bool RegistrationMap::GetScanInMapFrame(const ros::Time& stamp,
                                        PointCloud& cloud) const {
  auto iter = scans_.find(stamp.toNSec());
  if (iter != scans_.end()) {
    cloud = iter->second.cloud;
    return true;
  }

  return false;
}

bool RegistrationMap::GetScanInMapFrame(const ros::Time& stamp,
                                        LoamPointCloud& cloud) const {
  auto iter = scans_.find(stamp.toNSec());
  if (iter != scans_.end()) {
    cloud = iter->second.loam_cloud;
    return true;
  }

  return false;
}

bool RegistrationMap::GetUUIDStamp(const fuse_core::UUID& uuid,
                                   ros::Time& stamp) const {
  for (const auto& [t, scan] : scans_) {
    if (scan.position_uuid == uuid || scan.orientation_uuid == uuid) {
      stamp.fromNSec(t);
      return true;
    }
  }
  return false;
}

void RegistrationMap::Clear() {
  scans_.clear();
}

void RegistrationMap::Publish() {
  if (!publish_updates_) { return; }

  ros::Time update_time = ros::Time::now();

  // get maps
  PointCloud lidar_map = GetPointCloudMap();
  LoamPointCloud loam_map = GetLoamCloudMap();

  if (!lidar_map.empty()) {
    sensor_msgs::PointCloud2 pc_msg = beam::PCLToROS<pcl::PointXYZ>(
        lidar_map, update_time, world_frame_id_, updates_counter_);
    lidar_map_publisher_.publish(pc_msg);
  }

  if (!loam_map.Empty()) {
    LoamPointCloudCombined loam_combined = loam_map.GetCombinedCloud();
    sensor_msgs::PointCloud2 pc_msg = beam::PCLToROS<PointLoam>(
        loam_combined, update_time, world_frame_id_, updates_counter_);
    loam_map_publisher_.publish(pc_msg);
  }

  updates_counter_++;
}

ros::Time RegistrationMap::GetLastCloudPoseStamp() const {
  if (scans_.empty()) { return {}; }
  uint64_t t_in_ns = scans_.rbegin()->first;
  ros::Time stamp;
  stamp.fromNSec(t_in_ns);
  return stamp;
}

void RegistrationMap::UpdateScanPosesFromGraphMsg(
    const fuse_core::Graph::ConstSharedPtr& graph_msg) {
  for (const auto& [t_in_ns, scan] : scans_) {
    if (!graph_msg->variableExists(scan.position_uuid) ||
        !graph_msg->variableExists(scan.orientation_uuid)) {
      continue;
    }

    ros::Time stamp;
    stamp.fromNSec(t_in_ns);

    auto position = dynamic_cast<const fuse_variables::Position3DStamped&>(
        graph_msg->getVariable(scan.position_uuid));
    auto orientation =
        dynamic_cast<const fuse_variables::Orientation3DStamped&>(
            graph_msg->getVariable(scan.orientation_uuid));

    Eigen::Matrix4d T_World_Baselink;
    bs_common::FusePoseToEigenTransform(position, orientation,
                                        T_World_Baselink);

    bs_common::ExtrinsicsLookupOnline& extrinsics =
        bs_common::ExtrinsicsLookupOnline::GetInstance();
    Eigen::Matrix4d T_Baselink_Scan;
    extrinsics.GetT_BASELINK_LIDAR(T_Baselink_Scan);
    Eigen::Matrix4d T_World_Scan = T_World_Baselink * T_Baselink_Scan;
    UpdateScan(stamp, T_World_Scan);
  }
}

void RegistrationMap::CorrectMapDriftFromGraphMsg(
    const fuse_core::Graph::ConstSharedPtr& graph_msg) {
  Eigen::Matrix4d T_WorldCorrected_World;
  bool success = false;
  for (auto riter = scans_.rbegin(); riter != scans_.rend(); riter++) {
    if (!graph_msg->variableExists(riter->second.position_uuid) ||
        !graph_msg->variableExists(riter->second.orientation_uuid)) {
      continue;
    }

    ros::Time stamp;
    stamp.fromNSec(riter->first);

    auto position = dynamic_cast<const fuse_variables::Position3DStamped&>(
        graph_msg->getVariable(riter->second.position_uuid));
    auto orientation =
        dynamic_cast<const fuse_variables::Orientation3DStamped&>(
            graph_msg->getVariable(riter->second.orientation_uuid));

    Eigen::Matrix4d T_World_BaselinkCorrected;
    bs_common::FusePoseToEigenTransform(position, orientation,
                                        T_World_BaselinkCorrected);
    bs_common::ExtrinsicsLookupOnline& extrinsics =
        bs_common::ExtrinsicsLookupOnline::GetInstance();
    Eigen::Matrix4d T_Baselink_Scan;
    extrinsics.GetT_BASELINK_LIDAR(T_Baselink_Scan);
    T_WorldCorrected_World = T_World_BaselinkCorrected * T_Baselink_Scan *
                             beam::InvertTransform(riter->second.T_Map_Scan);
    success = true;
    break;
  }

  if (!success) {
    BEAM_ERROR("Cannot update registration map, no scan found in graph msg");
    return;
  }

  // update al poses
  for (auto& [t_in_ns, scan] : scans_) {
    scan.T_Map_Scan = T_WorldCorrected_World * scan.T_Map_Scan;
  }
}

}} // namespace bs_models::scan_registration