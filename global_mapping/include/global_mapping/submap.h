#pragma once


#include <beam_utils/pointclouds.h>
#include <beam_containers/LandmarkContainer.h>
#include <global_mapping/LandmarkMeasurement.h>
#include <beam_common/scan_pose.h>

#include <map>

#include <boost/filesystem.hpp>
#include <fuse_core/graph.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <ros/time.h>

namespace global_mapping {

using pose_allocator = Eigen::aligned_allocator<Eigen::Matrix4d>;

class Submap {
 public:
  /**
   * @brief constructor that requires a pose and stamp for the submap.
   * @param T_WORLD_SUBMAP pose in matrix form
   * @param stamp timestamp associated with the submap pose
   */
  Submap(const ros::Time& stamp, const Eigen::Matrix4d& T_WORLD_SUBMAP);

  /**
   * @brief constructor that requires a pose and stamp for the submap.
   * @param position t_WORLD_SUBMAP
   * @param orientation R_WORLD_SUBMAP
   * @param stamp timestamp associated with the submap pose
   */
  Submap(const ros::Time& stamp, const fuse_variables::Position3DStamped& position,
         const fuse_variables::Orientation3DStamped& orientation);

  /**
   * @brief default constructor
   */
  ~Submap() = default;

  /**
   * @brief get current position estimate
   * @return t_WORLD_SUBMAP
   */
  fuse_variables::Position3DStamped Position();

  /**
   * @brief get current orientation estimate
   * @return R_WORLD_SUBMAP
   */
  fuse_variables::Orientation3DStamped Orientation();

  /**
   * @brief get current transform estimate
   * @return T_WORLD_SUBMAP
   */
  Eigen::Matrix4d T_WORLD_SUBMAP();

  /**
   * @brief get initial pose estimate in matrix form
   * @return T_WORLD_SUBMAP
   */
  Eigen::Matrix4d T_WORLD_SUBMAP_INIT();

  /**
   * @brief get the number of pose updates from a pose graph optimization
   * @return number of updates
   */
  int Updates();

  /**
   * @brief get current time associated with the submap pose
   * @return stamp
   */
  ros::Time Stamp();

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
  void AddCameraMeasurement(const std::vector<LandmarkMeasurement>& landmarks,
                            const Eigen::Matrix4d& T_WORLD_FRAME,
                            const ros::Time& stamp, int sensor_id,
                            int measurement_id);

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
  bool Near(const ros::Time& time, const double tolerance);

  /**
   * @brief check if the submap was generated before or after some other submap
   * @return true if time of this submap is less than the time of the input
   * submap
   */
  bool operator<(const Submap& rhs);

  /**
   * @brief save all 3D keypoints in landmark measurements to a single
   * pointcloud map. Points will be converted to world frame before saving
   * @param filename filename to save to including full path
   */
  void SaveKeypointsMapInWorldFrame(const std::string& filename);

  /**
   * @brief save all lidar points to a single pointcloud map. Points will be
   * converted to world frame before saving
   * @param filename filename to save to including full path
   */
  void SaveLidarMapInWorldFrame(const std::string& filename);

  /**
   * @brief output all 3D keypoints in landmark measurements to a single
   * pointcloud. Points will be converted to world frame before outputting
   * @return cloud
   */
  PointCloud GetKeypointsInWorldFrame();

  /**
   * @brief output all lidar points to a single pointcloud map. Points will be
   * converted to world frame before outputting
   * @param return cloud
   */
  PointCloud GetLidarPointsInWorldFrame();

  /**
   * @brief print relevant information about what is currently contained in this
   * submap. Example: pose, number of lidar scans and keypoints, etc...
   * @param stream input stream
   */
  void Print(std::ostream& stream = std::cout);

 private:
  ros::Time stamp_;
  int graph_updates_{0};
  fuse_variables::Position3DStamped position_;        // t_WORLD_SUBMAP
  fuse_variables::Orientation3DStamped orientation_;  // R_WORLD_SUBMAP
  Eigen::Matrix4d T_WORLD_SUBMAP_initial_;
  std::map<double, beam_common::ScanPose> scans_poses_; // <time, scan pose>

  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  beam_containers::LandmarkContainer<beam_containers::LandmarkMeasurement>
      landmarks_;
  std::map<uint64_t, Eigen::Matrix4d> keyframe_poses_; // <time, pose>
  std::map<uint64_t, Eigen::Vector3d> landmark_positions_; // <id, position>
};

} // namespace global_mapping