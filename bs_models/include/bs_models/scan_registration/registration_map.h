#pragma once

#include <fuse_core/graph.h>
#include <fuse_core/uuid.h>
#include <ros/publisher.h>

#include <beam_matching/loam/LoamPointCloud.h>
#include <beam_utils/pointclouds.h>

namespace bs_models { namespace scan_registration {

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
class RegistrationMap {
public:
  struct ScanPoseInMapFrame {
    Eigen::Matrix4d T_Map_Scan;
    PointCloud cloud;
    LoamPointCloud loam_cloud;
    fuse_core::UUID orientation_uuid;
    fuse_core::UUID position_uuid;
  };

  /**
   * @brief Static Instance getter (singleton)
   * @return reference to the singleton
   */
  static RegistrationMap& GetInstance();

  /**
   * @brief set the parameters of this lidar map. Since this is implemented as a
   * singleton, this checks if the params were already set, if so then it'll
   * output a warning.
   * @param map_size number of scans to store in this map
   * @param publish_updates if set to true, this class with publish the full
   * lidar map in the world frame whenever the map is updated
   */
  bool SetParams(int map_size, bool publish_updates = false);

  /**
   * @brief return map size
   * @return map_size
   */
  int MapSize() const;

  /**
   * @brief checks if no scans have been added to this map
   * @return true if no clouds have been added (loam or regular pointclouds)
   */
  bool Empty() const;

  /**
   * @brief return the number of scans currently stored
   * @return number of scans stored
   */
  int NumScans() const;

  /**
   * @brief add pointcloud and convert to map frame. It will also remove
   * the first (chronologically) scan if the map has reached its max size.
   * @param cloud pointcloud to add (in some scan frame)
   * @param loam_cloud loam pointcloud to add (in some scan frame)
   * @param stamp timestamp associated with this scan. This is used to queue the
   * clouds in the map, which will remove the oldest maps once the map gets
   * larger than the max size
   * @param T_Map_Scan ransform from scan frame to map frame. This
   * will be applied to the scan before adding (to reduce computation, assuming
   * we will be frequently asking for the full map). The map frame is usually
   * the world frame.
   */
  void AddPointCloud(const PointCloud& cloud, const LoamPointCloud& loam_cloud,
                     const ros::Time& stamp, const Eigen::Matrix4d& T_Map_Scan);

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
   * @param T_Map_Scan updated scan pose
   * @param rotation_threshold_deg scans will only be updated if the new pose
   * has a rotation change greater than this threshold. Note we use the angle
   * part of Eigen::AxisAngle to determine difference in rotation.
   * @param translation_threshold_m scans will only be updated if the new pose
   * has a translation change greater than this threshold.
   * @return true if scan was updated. It will return false if the new pose is
   * too similar to the prev, or if the scan doesn't exist
   */
  bool UpdateScan(const ros::Time& stamp, const Eigen::Matrix4d& T_Map_Scan,
                  double rotation_threshold_deg = 0.5,
                  double translation_threshold_m = 0.005);

  /**
   * @brief save pointcloud of current scanposes
   * @param save_path full path to output directory. This directory must exist
   * @param add_frames whether or not to add coordinate frames from each scan
   * @param r red color intensity (for non loam clouds only)
   * @param g greeb color intensity (for non loam clouds only)
   * @param b blue color intensity (for non loam clouds only)
   */
  void Save(const std::string& save_path, bool add_frames = true,
            uint8_t r = 255, uint8_t g = 255, uint8_t b = 255) const;

  /**
   * @brief get a scan pose collected at some timestamp. This will check both
   * the loam cloud poses (this takes priority) and regular poses
   * @param stamp when scan was collected
   * @param T_Map_Scan reference to pose of the scan
   * @return true if scan with this timestamp exists
   */
  bool GetScanPose(const ros::Time& stamp, Eigen::Matrix4d& T_Map_Scan) const;

  /**
   * @brief get a scan collected at some timestamp, with points expressed in the
   * map frame
   * @param stamp when scan was collected
   * @param cloud reference to cloud to fill in
   * @return true if scan with thi stimestamp exists
   */
  bool GetScanInMapFrame(const ros::Time& stamp, PointCloud& cloud) const;

  /**
   * @brief get a scan collected at some timestamp, with points expressed in the
   * map frame
   * @param stamp when scan was collected
   * @param cloud reference to loam cloud to fill in
   * @return true if scan with thi stimestamp exists
   */
  bool GetScanInMapFrame(const ros::Time& stamp, LoamPointCloud& cloud) const;

  /**
   * @brief get the timestamp associated with a fuse uuid of position or
   * orientation. We store uuids every time we add a new scan so that we can
   * lookup scans or poses based on timestamp or uuid.
   * @param uuid uuid of position fuse variable or orientation fuse variable
   * @param stamp reference to stamp
   * @return true if scan with this pose uuid exists
   */
  bool GetUUIDStamp(const fuse_core::UUID& uuid, ros::Time& stamp) const;

  ros::Time GetLastCloudPoseStamp() const;

  /**
   * @brief update all scan poses in the map from a graph message. Only the
   * scans that are in the graph will update their poses
   */
  void UpdateScanPosesFromGraphMsg(
      const fuse_core::Graph::ConstSharedPtr& graph_msg);

  /**
   * @brief to correct for drift, we take the most recent scan pose in the
   * registration map that has a pose in the graph, calculate the difference
   * between the two poses, and apply that correction to all poses in the
   * registration map
   */
  void CorrectMapDriftFromGraphMsg(
      const fuse_core::Graph::ConstSharedPtr& graph_msg);

  /**
   * @brief clears all scans and their associated poses
   */
  void Clear();

  /**
   * @brief publish the current map. This gets called each time the map saves,
   * if publish_updates_ is set to true
   */
  void Publish();

  /**
   * @brief Delete copy constructor
   */
  RegistrationMap(RegistrationMap const&) = delete;

  /**
   * @brief Delete assignment constructor
   */
  void operator=(RegistrationMap const&) = delete;

private:
  /**
   * @brief constructor. Uses default member variables
   */
  RegistrationMap();

  // publisher
  ros::Publisher lidar_map_publisher_;
  ros::Publisher loam_map_publisher_;

  int map_size_{10};
  bool map_params_set_{false};
  int updates_counter_{0};
  bool publish_updates_{false};
  std::string world_frame_id_;

  std::map<uint64_t, ScanPoseInMapFrame> scans_;
};

}} // namespace bs_models::scan_registration
