#include <bs_models/scan_registration/registration_map.h>

#include <boost/filesystem.hpp>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <bs_common/extrinsics_lookup_online.h>

namespace bs_models { namespace scan_registration {

RegistrationMap::RegistrationMap() {
  bs_common::ExtrinsicsLookupOnline& extrinsics_online =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
  frame_id_ = extrinsics_online.GetWorldFrameId();

  ros::NodeHandle n;

  // setup publishers
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
    while (clouds_in_map_frame_.size() > map_size) {
      uint64_t first_scan_stamp = clouds_in_map_frame_.begin()->first;
      clouds_in_map_frame_.erase(first_scan_stamp);
      cloud_poses_.erase(first_scan_stamp);
    }
    while (loam_clouds_in_map_frame_.size() > map_size) {
      uint64_t first_scan_stamp = loam_clouds_in_map_frame_.begin()->first;
      loam_clouds_in_map_frame_.erase(first_scan_stamp);
      loam_cloud_poses_.erase(first_scan_stamp);
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
  return loam_clouds_in_map_frame_.empty() && clouds_in_map_frame_.empty();
}

int RegistrationMap::NumPointClouds() const {
  return clouds_in_map_frame_.size();
}

int RegistrationMap::NumLoamClouds() const {
  return loam_clouds_in_map_frame_.size();
}

void RegistrationMap::AddPointCloud(const PointCloud& cloud,
                                    const ros::Time& stamp,
                                    const Eigen::Matrix4d& T_MAP_SCAN) {
  // add cloud to map
  PointCloudPtr cloud_in_map_frame = std::make_shared<PointCloud>();
  pcl::transformPointCloud(cloud, *cloud_in_map_frame, T_MAP_SCAN);
  clouds_in_map_frame_.emplace(stamp.toNSec(), cloud_in_map_frame);

  // add pose
  cloud_poses_.emplace(stamp.toNSec(), T_MAP_SCAN);

  // generate and add uuids
  fuse_variables::Position3DStamped dummy_pos;
  fuse_variables::Orientation3DStamped dummy_or;
  auto sensor_id = fuse_core::uuid::NIL;
  uuid_map_.emplace(
      fuse_core::uuid::generate(dummy_pos.type(), stamp, sensor_id),
      stamp.toNSec());
  uuid_map_.emplace(
      fuse_core::uuid::generate(dummy_or.type(), stamp, sensor_id),
      stamp.toNSec());

  // remove cloud & pose if map is greater than max size
  if (clouds_in_map_frame_.size() > map_size_) {
    uint64_t first_scan_stamp = clouds_in_map_frame_.begin()->first;
    clouds_in_map_frame_.erase(first_scan_stamp);
    cloud_poses_.erase(first_scan_stamp);
  }

  Publish();
}

void RegistrationMap::AddPointCloud(const LoamPointCloud& cloud,
                                    const ros::Time& stamp,
                                    const Eigen::Matrix4d& T_MAP_SCAN) {
  // add cloud to map
  LoamPointCloudPtr cloud_in_map_frame =
      std::make_shared<LoamPointCloud>(cloud, T_MAP_SCAN);
  loam_clouds_in_map_frame_.emplace(stamp.toNSec(), cloud_in_map_frame);

  // add pose
  loam_cloud_poses_.emplace(stamp.toNSec(), T_MAP_SCAN);

  // generate and add uuids
  fuse_variables::Position3DStamped dummy_pos;
  fuse_variables::Orientation3DStamped dummy_or;
  auto sensor_id = fuse_core::uuid::NIL;
  uuid_map_.emplace(
      fuse_core::uuid::generate(dummy_pos.type(), stamp, sensor_id),
      stamp.toNSec());
  uuid_map_.emplace(
      fuse_core::uuid::generate(dummy_or.type(), stamp, sensor_id),
      stamp.toNSec());

  // remove cloud & pose if map is greater than max size
  if (loam_clouds_in_map_frame_.size() > map_size_) {
    uint64_t first_scan_stamp = loam_clouds_in_map_frame_.begin()->first;
    loam_clouds_in_map_frame_.erase(first_scan_stamp);
    loam_cloud_poses_.erase(first_scan_stamp);
  }

  Publish();
}

PointCloud RegistrationMap::GetPointCloudMap() const {
  PointCloud cloud;
  for (auto it = clouds_in_map_frame_.begin(); it != clouds_in_map_frame_.end();
       it++) {
    cloud += *(it->second);
  }
  return cloud;
}

LoamPointCloud RegistrationMap::GetLoamCloudMap() const {
  LoamPointCloud cloud;
  for (auto it = loam_clouds_in_map_frame_.begin();
       it != loam_clouds_in_map_frame_.end(); it++) {
    cloud.Merge(*(it->second));
  }
  return cloud;
}

bool RegistrationMap::UpdateScan(const ros::Time& stamp,
                                 const Eigen::Matrix4d& T_MAP_SCAN,
                                 double rotation_threshold_deg,
                                 double translation_threshold_m) {
  bool scan_found{false};
  uint64_t stamp_nsecs = stamp.toNSec();

  // update regular pointclouds
  auto pose_it = cloud_poses_.find(stamp_nsecs);
  if (pose_it != cloud_poses_.end()) {
    scan_found = true;

    // check poses are not too similar
    if (beam::ArePosesEqual(T_MAP_SCAN, pose_it->second, rotation_threshold_deg,
                            translation_threshold_m)) {
      return false;
    }

    // update pointcloud
    auto cloud_iter = clouds_in_map_frame_.find(stamp_nsecs);
    if (cloud_iter == clouds_in_map_frame_.end()) {
      BEAM_ERROR(
          "Missmatch between clouds and cloud poses. Not updating scan.");
      return false;
    }
    Eigen::Matrix4d T_MAPNEW_MAPOLD =
        T_MAP_SCAN * beam::InvertTransform(pose_it->second);
    pcl::transformPointCloud(*(cloud_iter->second), *(cloud_iter->second),
                             T_MAPNEW_MAPOLD);
  }

  // update loam pointclouds
  auto loam_pose_it = loam_cloud_poses_.find(stamp_nsecs);
  if (loam_pose_it != loam_cloud_poses_.end()) {
    scan_found = true;

    // check poses are not too similar
    if (beam::ArePosesEqual(T_MAP_SCAN, loam_pose_it->second,
                            rotation_threshold_deg, translation_threshold_m)) {
      return false;
    }

    // update loam pointcloud
    auto cloud_iter = loam_clouds_in_map_frame_.find(stamp_nsecs);
    if (cloud_iter == loam_clouds_in_map_frame_.end()) {
      BEAM_ERROR(
          "Missmatch between clouds and cloud poses. Not updating scan.");
      return false;
    }
    Eigen::Matrix4d T_MAPNEW_MAPOLD =
        T_MAP_SCAN * beam::InvertTransform(loam_pose_it->second);
    cloud_iter->second->TransformPointCloud(T_MAPNEW_MAPOLD);
  }

  Publish();
  return scan_found;
}

void RegistrationMap::Save(const std::string& save_path, bool add_frames,
                           uint8_t r, uint8_t g, uint8_t b) const {
  if (!boost::filesystem::exists(save_path)) {
    BEAM_ERROR("Invalid output path for RegistrationMap: {}", save_path);
    return;
  }

  if (!loam_clouds_in_map_frame_.empty()) {
    LoamPointCloud map = GetLoamCloudMap();
    map.SaveCombined(save_path, "registration_map_loam.pcd");
  }
  if (!clouds_in_map_frame_.empty()) {
    PointCloud map = GetPointCloudMap();
    PointCloudCol map_col = beam::ColorPointCloud(map, r, g, b);
    PointCloudCol frame = beam::CreateFrameCol();
    for (auto it = cloud_poses_.begin(); it != cloud_poses_.end(); it++) {
      const Eigen::Matrix4d& T_MAP_SCAN = it->second;
      beam::MergeFrameToCloud(map_col, frame, T_MAP_SCAN);
    }

    std::string error_message{};
    if (!beam::SavePointCloud<pcl::PointXYZRGB>(
            save_path + "registration_map.pcd", map_col,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
  }
}

bool RegistrationMap::GetScanPose(const ros::Time& stamp,
                                  Eigen::Matrix4d& T_MAP_SCAN) const {
  // check loam poses
  auto loam_iter = loam_cloud_poses_.find(stamp.toNSec());
  if (loam_iter != loam_cloud_poses_.end()) {
    T_MAP_SCAN = loam_iter->second;
    return true;
  }

  // check other poses
  auto iter = cloud_poses_.find(stamp.toNSec());
  if (iter != cloud_poses_.end()) {
    T_MAP_SCAN = iter->second;
    return true;
  }

  return false;
}

bool RegistrationMap::GetScanInMapFrame(const ros::Time& stamp,
                                        PointCloud& cloud) const {
  auto iter = clouds_in_map_frame_.find(stamp.toNSec());
  if (iter != clouds_in_map_frame_.end()) {
    cloud = *(iter->second);
    return true;
  }

  return false;
}

bool RegistrationMap::GetScanInMapFrame(const ros::Time& stamp,
                                        LoamPointCloud& cloud) const {
  auto iter = loam_clouds_in_map_frame_.find(stamp.toNSec());
  if (iter != loam_clouds_in_map_frame_.end()) {
    cloud = *(iter->second);
    return true;
  }

  return false;
}

bool RegistrationMap::GetUUIDStamp(const fuse_core::UUID& uuid,
                                   ros::Time& stamp) const {
  auto iter = uuid_map_.find(uuid);
  if (iter == uuid_map_.end()) { return false; }
  stamp.fromNSec(iter->second);
  return true;
}

void RegistrationMap::Clear() {
  clouds_in_map_frame_.clear();
  cloud_poses_.clear();
  loam_clouds_in_map_frame_.clear();
  loam_cloud_poses_.clear();
  uuid_map_.clear();
}

void RegistrationMap::Publish() {
  if (!publish_updates_) { return; }

  ros::Time update_time = ros::Time::now();

  // get maps
  PointCloud lidar_map = GetPointCloudMap();
  LoamPointCloud loam_map = GetLoamCloudMap();

  if (!lidar_map.empty()) {
    sensor_msgs::PointCloud2 pc_msg = beam::PCLToROS<pcl::PointXYZ>(
        lidar_map, update_time, frame_id_, updates_counter_);
    lidar_map_publisher_.publish(pc_msg);
  }

  if (!loam_map.Empty()) {
    LoamPointCloudCombined loam_combined = loam_map.GetCombinedCloud();
    sensor_msgs::PointCloud2 pc_msg = beam::PCLToROS<PointLoam>(
        loam_combined, update_time, frame_id_, updates_counter_);
    loam_map_publisher_.publish(pc_msg);
  }

  updates_counter_++;
}

ros::Time RegistrationMap::GetLastLoamPoseStamp() const {
  if (cloud_poses_.empty()) { return {}; }
  uint64_t t_in_ns = cloud_poses_.rbegin()->first;
  ros::Time stamp;
  stamp.fromNSec(t_in_ns);
  return stamp;
}

ros::Time RegistrationMap::GetLastCloudPoseStamp() const {
  if (loam_cloud_poses_.empty()) { return {}; }
  uint64_t t_in_ns = loam_cloud_poses_.rbegin()->first;
  ros::Time stamp;
  stamp.fromNSec(t_in_ns);
  return stamp;
}

}} // namespace bs_models::scan_registration