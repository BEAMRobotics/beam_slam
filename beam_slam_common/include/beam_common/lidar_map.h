#pragma once

#include <fuse_core/graph.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_utils/pointclouds.h>
#include <beam_matching/loam/LoamPointCloud.h>

namespace beam_common {

using namespace beam_matching;

/**
 * @brief class for building, maintaining, and storing a lidar map. This class
 * is implemented as a singleton because there should only be one lidar map
 * being built for each slam session, and it should be accessible by different
 * classes or sensor models. For example, ScanToMapRegistration may be building
 * this map and using it for generating frame to frame constraints, whereas the
 * sensor model that generates lidar-camera constraints will need to access a
 * lidar map of its local environment. So to remove computation duplication we
 * can use the map we are already generating with the scan to map registration.
 */
class LidarMap {
 public:
  /**
   * @brief Static Instance getter (singleton)
   * @return reference to the singleton
   */
  static LidarMap& GetInstance() {
    static LidarMap instance;
    return instance;
  }

  /**
   * @brief set the parameters of this lidar map. Since this is implemented as a
   * singleton, we don't want to set the parameters from one client, then have
   * another change it. So this checks if the params were already set, if so
   * then it'll disregard this call and output a warning.
   * @param map_size number of scans to store in this map
   */
  bool SetParams(int map_size) {
    if (map_params_set_ && map_size != map_size_) {
      BEAM_ERROR("Map parameters already set, these cannot be changed.");
      return false;
    }

    map_size_ = map_size;
    map_params_set_ = true;
    return true;
  }

  /**
   * @brief checks if no pointclouds have been added to this map
   * @return true if no clouds have been added (loam or regular pointclouds)
   */
  bool Empty() const {
    return loam_clouds_in_map_frame_.empty() && clouds_in_map_frame_.empty();
  }

  /**
   * @brief return the number of pointclouds currently stored
   * @return number of pcl pointclouds stored
   */
  int NumPointClouds() const { return clouds_in_map_frame_.size(); }

  /**
   * @brief return the number of loam pointclouds currently stored
   * @return number of loam pointclouds stored
   */
  int NumLoamClouds() const { return loam_clouds_in_map_frame_.size(); }

  /**
   * @brief add pointcloud and convert to map frame. It will also remove
   * the first (chronologically) scan if the map has reached its max size.
   * @param cloud pointcloud to add (in some scan frame)
   * @param stamp timestamp associated with this scan. This is used to queue the
   * clouds in the map, which will remove the oldest maps once the map gets
   * larger than the max size
   * @param T_MAP_SCAN ransform from scan frame to map frame. This
   * will be applied to the scan before adding (to reduce computation, assuming
   * we will be frequently asking for the full map).
   */
  void AddPointCloud(const PointCloud& cloud, const ros::Time& stamp,
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

  /**
   * @brief add loam pointcloud and convert to map frame. It will also remove
   * the first (chronologically) scan if the map has reached its max size.
   * @param cloud loam pointcloud to add (in some scan frame)
   * @param stamp timestamp associated with this scan. This is used to queue the
   * clouds in the map, which will remove the oldest maps once the map gets
   * larger than the max size
   * @param T_MAP_SCAN transform from scan frame to map frame. This
   * will be applied to the scan before adding (to reduce computation, assuming
   * we will be frequently asking for the full map).
   */
  void AddPointCloud(const LoamPointCloud& cloud, const ros::Time& stamp,
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

  /**
   * @brief returns combined pointcloud in map frame
   * @return pointcloud
   */
  PointCloud GetPointCloudMap() const {
    PointCloud cloud;
    for (auto it = clouds_in_map_frame_.begin();
         it != clouds_in_map_frame_.end(); it++) {
      cloud += it->second;
    }
    return cloud;
  }

  /**
   * @brief returns combined loam pointcloud in map frame
   * @return LoamPointCloud
   */
  LoamPointCloud GetLoamCloudMap() const {
    LoamPointCloud cloud;
    for (auto it = loam_clouds_in_map_frame_.begin();
         it != loam_clouds_in_map_frame_.end(); it++) {
      cloud.Merge(it->second);
    }
    return cloud;
  }

  /**
   * @brief save pointcloud of current scanpose
   * @param save_path full path to output directory. This directory must exist
   * @param add_frames whether or not to add coordinate frames from each scan
   * @param r red color intensity (for non loam clouds only)
   * @param g greeb color intensity (for non loam clouds only)
   * @param b blue color intensity (for non loam clouds only)
   */
  void Save(const std::string& save_path, bool add_frames = true,
            uint8_t r = 255, uint8_t g = 255, uint8_t b = 255) const {
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

  /**
   * @brief Delete copy constructor
   */
  LidarMap(LidarMap const&) = delete;

  /**
   * @brief Delete assignment constructor
   */
  void operator=(LidarMap const&) = delete;

 private:
  /**
   * @brief default constructor. Uses default member variables
   */
  LidarMap() = default;

  std::map<uint64_t, LoamPointCloud> loam_clouds_in_map_frame_;
  std::map<uint64_t, Eigen::Matrix4d> loam_cloud_poses_;
  std::map<uint64_t, PointCloud> clouds_in_map_frame_;
  std::map<uint64_t, Eigen::Matrix4d> cloud_poses_;
  int map_size_{10};
  bool map_params_set_{false};
};

}  // namespace beam_common
