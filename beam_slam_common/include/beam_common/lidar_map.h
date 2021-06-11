#pragma once

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
  static LidarMap& GetInstance();

  /**
   * @brief set the parameters of this lidar map. Since this is implemented as a
   * singleton, we don't want to set the parameters from one client, then have
   * another change it. So this checks if the params were already set, if so
   * then it'll disregard this call and output a warning.
   * @param map_size number of scans to store in this map
   */
  bool SetParams(int map_size);

  /**
   * @brief checks if no pointclouds have been added to this map
   * @return true if no clouds have been added (loam or regular pointclouds)
   */
  bool Empty() const;

  /**
   * @brief return the number of pointclouds currently stored
   * @return number of pcl pointclouds stored
   */
  int NumPointClouds() const;

  /**
   * @brief return the number of loam pointclouds currently stored
   * @return number of loam pointclouds stored
   */
  int NumLoamClouds() const;

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
                     const Eigen::Matrix4d& T_MAP_SCAN);

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
                     const Eigen::Matrix4d& T_MAP_SCAN);

  /**
   * @brief returns combined pointcloud in map frame
   * @return pointcloud
   */
  PointCloud GetPointCloudMap() const;

  /**
   * @brief returns combined loam pointcloud in map frame
   * @return LoamPointCloud
   */
  LoamPointCloud GetLoamCloudMap() const;

  /**
   * @brief Updates all points in a scan if that scan is currently saved in the
   * map. It does this by checking for the timestmap in the loam_cloud_poses_ &
   * cloud_poses_ maps. It also checks that the poses have changed by some
   * minimum amount before iterating through all points and updating their
   * positions.
   * @param stamp time associated with the scan. This will be used to determine
   * if the scan is current saved in the map or not.
   * @param T_MAP_SCAN updated scan pose
   * @param rotation_threshold_deg scans will only be updated if the new pose
   * has a rotation change greater than this threshold. Note we use the angle
   * part of Eigen::AxisAngle to determine difference in rotation.
   * @param translation_threshold_m scans will only be updated if the new pose
   * has a translation change greater than this threshold.
   * @return true if scan was updated. It will return false if the new pose is
   * too similar to the prev, or if the scan doesn't exist
   */
  bool UpdateScan(const ros::Time& stamp, const Eigen::Matrix4d& T_MAP_SCAN,
                  double rotation_threshold_deg = 0.5,
                  double translation_threshold_m = 0.005);

  /**
   * @brief save pointcloud of current scanpose
   * @param save_path full path to output directory. This directory must exist
   * @param add_frames whether or not to add coordinate frames from each scan
   * @param r red color intensity (for non loam clouds only)
   * @param g greeb color intensity (for non loam clouds only)
   * @param b blue color intensity (for non loam clouds only)
   */
  void Save(const std::string& save_path, bool add_frames = true,
            uint8_t r = 255, uint8_t g = 255, uint8_t b = 255) const;

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

  std::map<uint64_t, PointCloud> clouds_in_map_frame_;
  std::map<uint64_t, Eigen::Matrix4d> cloud_poses_;
  std::map<uint64_t, LoamPointCloud> loam_clouds_in_map_frame_;
  std::map<uint64_t, Eigen::Matrix4d> loam_cloud_poses_;
  int map_size_{10};
  bool map_params_set_{false};
};

}  // namespace beam_common
