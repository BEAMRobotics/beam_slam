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
 * All data should be expressed in one "baselink" sensor frame
 * (usually set to imu frame), and with respect to the submap frame (not
 * world frame). This is made to be used in conjuction with GlobalMap which
 * allows for the relative pose of the submaps to be changed without affecting
 * the local consistency of the data in the submaps.
 *
 * To keep the local mapper running fast, we do not update the poses/points in
 * the local mapper when the global mapper closes loops. Instead, we keep track
 * of the original world frame in the local mapper, and when we pass data
 * between the two we make sure to transform to the appropriate world frame.
 * That transformation gets taken care of in the submaps, since submaps store
 * their original pose w.r.t. the world frame (in other words, the world frame
 * tracked by the local mapper)
 */
class Submap {
 public:
  struct PoseStamped {
    ros::Time stamp;
    Eigen::Matrix4d T_SUBMAP_BASELINK;
  };

  /**
   * @brief constructor that requires a pose and stamp for the submap.
   * @param T_WORLD_SUBMAP pose in matrix form
   * @param stamp timestamp associated with the submap pose
   * @param extrinsics extrinsics lookup object
   * @param camera_model camera model, used for kepoint triangulation and BA
   */
  Submap(const ros::Time& stamp, const Eigen::Matrix4d& T_WORLD_SUBMAP,
         const std::shared_ptr<ExtrinsicsLookup>& extrinsics,
         const std::shared_ptr<beam_calibration::CameraModel>& camera_model);

  /**
   * @brief constructor that requires a pose and stamp for the submap.
   * @param position t_WORLD_SUBMAP
   * @param orientation R_WORLD_SUBMAP
   * @param stamp timestamp associated with the submap pose
   * @param extrinsics extrinsics lookup object
   * @param camera_model camera model, used for kepoint triangulation and BA
   */
  Submap(const ros::Time& stamp,
         const fuse_variables::Position3DStamped& position,
         const fuse_variables::Orientation3DStamped& orientation,
         const std::shared_ptr<ExtrinsicsLookup>& extrinsics,
         const std::shared_ptr<beam_calibration::CameraModel>& camera_model);

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
   * @param T_WORLDLM_BASELINK pose of baselink at this camera measurement time.
   * Note this transform is relative to the original world estimate that is
   * tracked by the local mapper, not the optimized location from the PGO.
   * @param stamp stamp associated with the image frame
   * @param sensor_id used to lookup transforms
   * @param measurement_id id of this specific measurement (image)
   */
  void AddCameraMeasurement(
      const std::vector<LandmarkMeasurementMsg>& landmarks,
      const Eigen::Matrix4d& T_WORLD_BASELINK, const ros::Time& stamp,
      int sensor_id, int measurement_id);

  /**
   * @brief add a set of lidar measurements associated with one scan
   * @param cloud pointcloud in the baselink frame
   * @param T_WORLDLM_BASELINK pose of the baselink at this lidar scan. Note
   * this is relative to the original world estimate that is tracked by the
   * local mapper, not the optimized location from the PGO.
   * @param stamp stamp associated with the lidar scan
   * @param type type of lidar points. See description in LidarMeasurement.msg
   */
  void AddLidarMeasurement(const PointCloud& cloud,
                           const Eigen::Matrix4d& T_WORLDLM_BASELINK,
                           const ros::Time& stamp, int type);

  /**
   * @brief add a set of trajectory measurements associated with some lidar or
   * image keyframe
   * @param poses set of poses relative to the keyframe (T_KEYFRAME_FRAME)
   * describing the trajectory between some less freqent measurement (i.e. lidar
   * or camera). Note that keyframe and frame are both baselink frames.
   * @param stamps set of time stamps associated with the poses above
   * @param stamp stamp associated with the keyframe that this trajectory is
   * attached to.
   */
  void AddTrajectoryMeasurement(
      const std::vector<Eigen::Matrix4d, pose_allocator>& poses,
      const std::vector<ros::Time>& stamps, const ros::Time& stamp);

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
   * @param use_initial_world_frame set to true to use the initial world frame
   * from the local mapper, before global optimization
   */
  void SaveKeypointsMapInWorldFrame(const std::string& filename,
                                    bool use_initial_world_frame = false);

  /**
   * @brief save all lidar points to a single pointcloud map. Points will be
   * converted to world frame before saving
   * @param filename filename to save to including full path
   * @param use_initial_world_frame set to true to use the initial world frame
   * from the local mapper, before global optimization
   */
  void SaveLidarMapInWorldFrame(const std::string& filename,
                                bool use_initial_world_frame = false) const;

  /**
   * @brief save all lidar loam pointclouds. Points will be
   * converted to world frame before saving. Separate clouds will be stored for
   * each feature type, with the option to also combine them
   * @param path full path to output directory (cannot specify filename since
   * multiple files will be outputted)
   * @param combine_features set to true to also output a combined map of all
   * features
   * @param use_initial_world_frame set to true to use the initial world frame
   * from the local mapper, before global optimization
   */
  void SaveLidarLoamMapInWorldFrame(const std::string& path,
                                    bool combine_features = true,
                                    bool use_initial_world_frame = false) const;

  /**
   * @brief output all 3D keypoints in landmark measurements to a single
   * pointcloud. Points will be converted to world frame before outputting
   * @param use_initial_world_frame set to true to use the initial world frame
   * from the local mapper, before global optimization
   * @return cloud
   */
  PointCloud GetKeypointsInWorldFrame(bool use_initial_world_frame = false);

  /**
   * @brief output all lidar points to a single pointcloud map. Points will be
   * converted to world frame before outputting
   * @param use_initial_world_frame set to true to use the initial world frame
   * from the local mapper, before global optimization
   * @param return cloud
   */
  PointCloud GetLidarPointsInWorldFrame(
      bool use_initial_world_frame = false) const;

  /**
   * @brief output all lidar LOAM points to a single pointcloud map. Points will
   * be converted to world frame before outputting
   * @param use_initial_world_frame set to true to use the initial world frame
   * from the local mapper, before global optimization
   * @param return cloud
   */
  beam_matching::LoamPointCloud GetLidarLoamPointsInWorldFrame(
      bool use_initial_world_frame = false) const;

  /**
   * @brief return a vector of stamped poses for all keyframes and their
   * attached sub-trajectories
   * @param return vectors of stamped poses
   */
  std::vector<Submap::PoseStamped> GetTrajectory() const;

  /**
   * @brief print relevant information about what is currently contained in this
   * submap. Example: pose, number of lidar scans and keypoints, etc...
   * @param stream input stream
   */
  void Print(std::ostream& stream = std::cout) const;

 private:
  /**
   * @brief Get 3D positions of each landmark given current tracks and camera
   * poses. This fills in landmark_positions_
   * @param override_points if set to false, it will skip triangulation if it
   * has already been called
   */
  void TriangulateKeypoints(bool override_points = false);

  // general submap data
  ros::Time stamp_;
  int graph_updates_{0};
  fuse_variables::Position3DStamped position_;        // t_WORLD_SUBMAP
  fuse_variables::Orientation3DStamped orientation_;  // R_WORLD_SUBMAP
  std::shared_ptr<ExtrinsicsLookup> extrinsics_;
  Eigen::Matrix4d T_WORLD_SUBMAP_;  // this get recomputed when fuse vars change
  Eigen::Matrix4d T_WORLD_SUBMAP_initial_;  // = T_WORLDLM_SUBMAP
  Eigen::Matrix4d T_SUBMAP_WORLD_initial_;  // = T_SUBMAP_WORLDLM

  // lidar data
  std::map<uint64_t, ScanPose> lidar_keyframe_poses_;  // <time,ScanPose>

  // camera data
  std::shared_ptr<beam_calibration::CameraModel> camera_model_;
  std::map<uint64_t, Eigen::Matrix4d> camera_keyframe_poses_;  // <time, pose>
  std::map<uint64_t, Eigen::Vector3d> landmark_positions_;     // <id, position>
  beam_containers::LandmarkContainer<beam_containers::LandmarkMeasurement>
      landmarks_;

  // subframe trajectory measurements
  std::map<uint64_t, std::vector<PoseStamped>> subframe_poses_;

  // NOTE: all poses are T_SUBMAP_BASELINK
};

}  // namespace global_mapping