#pragma once

#include <map>

#include <boost/filesystem.hpp>
#include <fuse_core/graph.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <ros/time.h>

#include <beam_utils/pointclouds.h>
#include <beam_containers/LandmarkContainer.h>
#include <beam_containers/LandmarkMeasurement.h>
#include <beam_calibration/CameraModel.h>
#include <beam_matching/loam/LoamPointCloud.h>
#include <global_mapping/LandmarkMeasurementMsg.h>
#include <beam_common/scan_pose.h>
#include <beam_common/extrinsics_lookup.h>

namespace global_mapping {

using pose_allocator = Eigen::aligned_allocator<Eigen::Matrix4d>;
using namespace beam_common;

/**
 * @brief class for holding and performing operation on locally consistent SLAM
 * data chunks.
 *
 * NOTE: All data should be expressed in one "baselink" sensor frame
 * (usually set to camera frame), and with respect to the submap frame (not
 * world frame). This is made to be used in conjuction with GlobalMap which
 * allows for the relative pose of the submaps to be changed without affecting
 * the local consistency of the data in the submaps.
 */
class Submap {
 public:
  struct PoseStamped {
    ros::Time stamp;
    Eigen::Matrix4d T_SUBMAP_SENSOR;
  };

  /**
   * @brief constructor that requires a pose and stamp for the submap.
   * @param T_WORLD_SUBMAP pose in matrix form
   * @param stamp timestamp associated with the submap pose
   */
  Submap(const ros::Time& stamp, const Eigen::Matrix4d& T_WORLD_SUBMAP,
         const std::shared_ptr<ExtrinsicsLookup>& extrinsics);

  /**
   * @brief constructor that requires a pose and stamp for the submap.
   * @param position t_WORLD_SUBMAP
   * @param orientation R_WORLD_SUBMAP
   * @param stamp timestamp associated with the submap pose
   */
  Submap(const ros::Time& stamp,
         const fuse_variables::Position3DStamped& position,
         const fuse_variables::Orientation3DStamped& orientation,
         const std::shared_ptr<ExtrinsicsLookup>& extrinsics);

  /**
   * @brief default destructor
   */
  ~Submap() = default;

  /**
   * @brief get current position estimate
   * @return t_WORLD_SUBMAP
   */
  fuse_variables::Position3DStamped Position() const;

  /**
   * @brief get current orientation estimate
   * @return R_WORLD_SUBMAP
   */
  fuse_variables::Orientation3DStamped Orientation() const;

  /**
   * @brief get current transform estimate
   * @return T_WORLD_SUBMAP
   */
  Eigen::Matrix4d T_WORLD_SUBMAP() const;

  /**
   * @brief get initial pose estimate in matrix form
   * @return T_WORLD_SUBMAP
   */
  Eigen::Matrix4d T_WORLD_SUBMAP_INIT() const;

  /**
   * @brief get the number of pose updates from a pose graph optimization
   * @return number of updates
   */
  int Updates() const;

  /**
   * @brief get current time associated with the submap pose
   * @return stamp
   */
  ros::Time Stamp() const;

  /**
   * @brief add a set of camera measurements associated with one image frame
   * @param landmarks landmark measurements viewed by the input frame
   * @param T_WORLD_FRAME pose of the frame. Note this is relative to the
   * original world estimate that is tracked by the local mapper, not the
   * optimized location from the PGO.
   * @param stamp stamp associated with the image frame
   * @param sensor_id used to lookup transforms
   * @param measurement_id id of this specific measurement (image)
   */
  void AddCameraMeasurement(
      const std::vector<LandmarkMeasurementMsg>& landmarks,
      const Eigen::Matrix4d& T_WORLD_FRAME, const ros::Time& stamp,
      int sensor_id, int measurement_id);

  /**
   * @brief add a set of lidar measurements associated with one scan
   * @param cloud pointcloud in some scan frame
   * @param T_WORLD_FRAME pose of the lidar scan (or frame). Note this is
   * relative to the original world estimate that is tracked by the local
   * mapper, not the optimized location from the PGO.
   * @param stamp stamp associated with the lidar scan
   * @param sensor_id used to lookup transforms
   * @param measurement_id id of this specific measurement (scan)
   * @param type type of lidar points. See description in LidarMeasurement.msg
   */
  void AddLidarMeasurement(const PointCloud& cloud,
                           const Eigen::Matrix4d& T_WORLD_FRAME,
                           const ros::Time& stamp, int sensor_id,
                           int measurement_id, int type);

  /**
   * @brief add a set of trajectory measurements associated with some frame
   * (usually relative poses to an image keyframe)
   * @param poses set of poses (T_WORLD_FRAME) describing the trajectory between
   * some less freqent measurement (i.e. lidar or camera)
   * @param stamps set of time stamps associated with the poses above
   * @param T_WORLD_FRAME pose of the lidar scan (or frame). Note this is
   * relative to the original world estimate that is tracked by the local
   * mapper, not the optimized location from the PGO.
   * @param stamp stamp associated with the lidar scan
   * @param sensor_id used to lookup transforms
   * @param measurement_id id of this specific measurement (scan)
   */
  void AddTrajectoryMeasurement(
      const std::vector<Eigen::Matrix4d, pose_allocator>& poses,
      const std::vector<ros::Time>& stamps,
      const Eigen::Matrix4d& T_WORLD_FRAME, const ros::Time& stamp,
      int sensor_id, int measurement_id);

  /**
   * @brief update the submap pose with an updated graph message
   * @param graph_msg new graph message that should contain the submap pose
   * variables
   */
  bool UpdatePose(fuse_core::Graph::ConstSharedPtr graph_msg);

  /**
   * @brief check if submap time is within some range of another timestamp
   * @param time query time
   * @param tolerance max time difference for this to return true
   * @return true if query time difference is within some tolorance
   */
  bool Near(const ros::Time& time, const double tolerance) const;

  /**
   * @brief check if the submap was generated before or after some other submap
   * @return true if time of this submap is less than the time of the input
   * submap
   */
  bool operator<(const Submap& rhs) const;

  /**
   * @brief save all 3D keypoints in landmark measurements to a single
   * pointcloud map. Points will be converted to world frame before saving
   * @param filename filename to save to including full path
   */
  void SaveKeypointsMapInWorldFrame(const std::string& filename) const;

  /**
   * @brief save all lidar points to a single pointcloud map. Points will be
   * converted to world frame before saving
   * @param filename filename to save to including full path
   */
  void SaveLidarMapInWorldFrame(const std::string& filename) const;

  /**
   * @brief output all 3D keypoints in landmark measurements to a single
   * pointcloud. Points will be converted to world frame before outputting
   * @return cloud
   */
  PointCloud GetKeypointsInWorldFrame() const;

  /**
   * @brief output all lidar points to a single pointcloud map. Points will be
   * converted to world frame before outputting
   * @param return cloud
   */
  PointCloud GetLidarPointsInWorldFrame() const;

  /**
   * @brief output all lidar LOAM points to a single pointcloud map. Points will
   * be converted to world frame before outputting
   * @param return cloud
   */
  beam_matching::LoamPointCloud GetLidarLoamPointsInWorldFrame() const;

  /**
   * @brief return a vector of stamped poses for all camera keyframes and their
   * attached sub-trajectories
   * @param return vectors of stamped poses
   */
  std::vector<PoseStamped> GetCameraTrajectory() const;

  /**
   * @brief return a vector of stamped poses for all lidar keyframes and their
   * attached sub-trajectories
   * @param return vectors of stamped poses
   */
  std::vector<PoseStamped> GetLidarTrajectory() const;

  /**
   * @brief print relevant information about what is currently contained in this
   * submap. Example: pose, number of lidar scans and keypoints, etc...
   * @param stream input stream
   */
  void Print(std::ostream& stream = std::cout) const;

 private:
  // general submap data
  ros::Time stamp_;
  int graph_updates_{0};
  fuse_variables::Position3DStamped position_;        // t_WORLD_SUBMAP
  fuse_variables::Orientation3DStamped orientation_;  // R_WORLD_SUBMAP
  std::shared_ptr<ExtrinsicsLookup> extrinsics_;
  Eigen::Matrix4d T_WORLD_SUBMAP_initial_;

  // lidar data
  std::map<uint64_t, ScanPose> lidar_keyframe_poses_;  // <time,ScanPose>
  std::map<uint64_t, PoseStamped> lidar_subframe_poses_;

  // camera data
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::map<uint64_t, Eigen::Matrix4d> camera_keyframe_poses_;  // <time, pose>
  std::map<uint64_t, PoseStamped> camera_subframe_poses_;
  //   std::map<uint64_t, Eigen::Vector3d> landmark_positions_;  // <id,
  //   position>
  beam_containers::LandmarkContainer<beam_containers::LandmarkMeasurement>
      landmarks_;
};

}  // namespace global_mapping