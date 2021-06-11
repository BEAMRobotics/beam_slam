#include <beam_common/lidar_map.h>

#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

namespace beam_common {

LidarMap& LidarMap::GetInstance() {
  static LidarMap instance;
  return instance;
}

bool LidarMap::SetParams(int map_size) {
  if (map_params_set_ && map_size != map_size_) {
    BEAM_ERROR("Map parameters already set, these cannot be changed.");
    return false;
  }

  map_size_ = map_size;
  map_params_set_ = true;
  return true;
}

bool LidarMap::Empty() const {
  return loam_clouds_in_map_frame_.empty() && clouds_in_map_frame_.empty();
}

int LidarMap::NumPointClouds() const { return clouds_in_map_frame_.size(); }

int LidarMap::NumLoamClouds() const { return loam_clouds_in_map_frame_.size(); }

void LidarMap::AddPointCloud(const PointCloud& cloud, const ros::Time& stamp,
                             const Eigen::Matrix4d& T_MAP_SCAN) {
  // add cloud to map
  PointCloud cloud_in_map_frame;
  pcl::transformPointCloud(cloud, cloud_in_map_frame, T_MAP_SCAN);
  clouds_in_map_frame_.emplace(stamp.toNSec(), cloud_in_map_frame);

  // add pose
  cloud_poses_.emplace(stamp.toNSec(), T_MAP_SCAN);

  // remove cloud & pose if map is greater than max size
  if (clouds_in_map_frame_.size() > map_size_) {
    uint64_t first_scan_stamp = clouds_in_map_frame_.begin()->first;
    clouds_in_map_frame_.erase(first_scan_stamp);
    cloud_poses_.erase(first_scan_stamp);
  }
}

void LidarMap::AddPointCloud(const LoamPointCloud& cloud,
                             const ros::Time& stamp,
                             const Eigen::Matrix4d& T_MAP_SCAN) {
  // add cloud to map
  LoamPointCloud cloud_in_map_frame = cloud;
  cloud_in_map_frame.TransformPointCloud(T_MAP_SCAN);
  loam_clouds_in_map_frame_.emplace(stamp.toNSec(), cloud_in_map_frame);

  // add pose
  loam_cloud_poses_.emplace(stamp.toNSec(), T_MAP_SCAN);

  // remove cloud & pose if map is greater than max size
  if (loam_clouds_in_map_frame_.size() > map_size_) {
    uint64_t first_scan_stamp = loam_clouds_in_map_frame_.begin()->first;
    loam_clouds_in_map_frame_.erase(first_scan_stamp);
    loam_cloud_poses_.erase(first_scan_stamp);
  }
}

PointCloud LidarMap::GetPointCloudMap() const {
  PointCloud cloud;
  for (auto it = clouds_in_map_frame_.begin(); it != clouds_in_map_frame_.end();
       it++) {
    cloud += it->second;
  }
  return cloud;
}

LoamPointCloud LidarMap::GetLoamCloudMap() const {
  LoamPointCloud cloud;
  for (auto it = loam_clouds_in_map_frame_.begin();
       it != loam_clouds_in_map_frame_.end(); it++) {
    cloud.Merge(it->second);
  }
  return cloud;
}

bool LidarMap::UpdateScan(const ros::Time& stamp,
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
    PointCloud cloud_updated;
    pcl::transformPointCloud(cloud_iter->second, cloud_updated,
                             T_MAPNEW_MAPOLD);
    cloud_iter->second = cloud_updated;
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
    cloud_iter->second.TransformPointCloud(T_MAPNEW_MAPOLD);
  }

  return scan_found;
}

void LidarMap::Save(const std::string& save_path, bool add_frames, uint8_t r,
                    uint8_t g, uint8_t b) const {
  if (!boost::filesystem::exists(save_path)) {
    BEAM_ERROR("Invalid output path for LidarMap: {}", save_path);
    return;
  }

  if (!loam_clouds_in_map_frame_.empty()) {
    LoamPointCloud map = GetLoamCloudMap();
    map.Save(save_path);
  }
  if (!clouds_in_map_frame_.empty()) {
    PointCloud map = GetPointCloudMap();
    PointCloudCol map_col = beam::ColorPointCloud(map, r, g, b);
    PointCloudCol frame = beam::CreateFrameCol();
    for (auto it = cloud_poses_.begin(); it != cloud_poses_.end(); it++) {
      const Eigen::Matrix4d& T_MAP_SCAN = it->second;
      beam::MergeFrameToCloud(map_col, frame, T_MAP_SCAN);
    }
    pcl::io::savePCDFileASCII(save_path + "pointcloud.pcd", map_col);
  }
}

}  // namespace beam_common