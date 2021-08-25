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
#include <bs_common/bs_msgs.h>
#include <bs_common/scan_pose.h>
#include <bs_common/extrinsics_lookup_online.h>

namespace bs_models {

namespace global_mapping {

using pose_allocator = Eigen::aligned_allocator<Eigen::Matrix4d>;
using namespace bs_common;

/**
 * @brief class for holding and performing operation on locally consistent SLAM
 * data chunks.
 *
 * Frame Convention:
 * -----------------
 *
 * All poses should be expressed in one "baselink" sensor frame
 * (usually set to imu frame) based on what is set in the extrinsics object.
 *
 * Each pose is also with respect to the submap frame (not world frame).
 *
 * Submaps are made to be used in conjuction with GlobalMap which allows for the
 * relative pose of the submaps to be changed without affecting the local
 * consistency of the data in the submaps.
 *
 * Lidar poses, however, are stored as transforms from lidar to submap since it
 * is more convenient to store the lidar data in the lidar sensor's frame. This
 * is needed for extrinsic calibration.
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
    Eigen::Matrix4d pose;  // Either T_KEYFRAME_FRAME, or T_SUBMAP_FRAME
  };

  /**
   * @brief constructor that requires a pose and stamp for the submap.
   * @param T_WORLD_SUBMAP pose in matrix form
   * @param stamp timestamp associated with the submap pose
   * @param camera_model camera model, used for kepoint triangulation and BA
   * @param extrinsics pointer to extrinsics. This can be updated by the source
   * if these are changing
   */
  Submap(const ros::Time& stamp, const Eigen::Matrix4d& T_WORLD_SUBMAP,
         const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
         const std::shared_ptr<bs_common::ExtrinsicsLookupBase>& extrinsics);

  /**
   * @brief constructor that requires a pose and stamp for the submap.
   * @param position t_WORLD_SUBMAP
   * @param orientation R_WORLD_SUBMAP
   * @param stamp timestamp associated with the submap pose
   * @param camera_model camera model, used for kepoint triangulation and BA
   * @param extrinsics pointer to extrinsics. This can be updated by the source
   * if these are changing
   */
  Submap(const ros::Time& stamp,
         const fuse_variables::Position3DStamped& position,
         const fuse_variables::Orientation3DStamped& orientation,
         const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
         const std::shared_ptr<bs_common::ExtrinsicsLookupBase>& extrinsics);

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
   * @param descriptor_type_int see DescriptorTypeIntMap in
   * beam_cv/descriptors/Descriptor.h
   * @param T_WORLDLM_BASELINK pose of baselink at this camera measurement time.
   * Note this transform is relative to the original world estimate that is
   * tracked by the local mapper, not the optimized location from the PGO.
   * @param stamp stamp associated with the image frame
   * @param sensor_id camera sensor id. This isn't really used because we
   * currently only use one camera
   * @param measurement_id id of this specific measurement (image)
   */
  void AddCameraMeasurement(
      const std::vector<LandmarkMeasurementMsg>& landmarks,
      uint8_t descriptor_type_int, const Eigen::Matrix4d& T_WORLD_BASELINK,
      const ros::Time& stamp, int sensor_id, int measurement_id);

  /**
   * @brief add a set of lidar measurements associated with one scan
   * @param cloud pointcloud in the lidar frame
   * @param T_WORLDLM_BASELINK pose of the baselink at this lidar scan time.
   * Note this is relative to the original world estimate that is tracked by the
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
   * attached to
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
   * @param max_output_map_size this function will convert all submap lidar
   * points to the world frame, but it's possible the map still gets too large,
   * so this will break it up
   * @param use_initial_world_frame set to true to use the initial world frame
   * from the local mapper, before global optimization
   */
  void SaveLidarMapInWorldFrame(const std::string& filename,
                                int max_output_map_size,
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
   * @brief output all lidar points to a vector of pointcloud maps. Points will
   * be converted to world frame before outputting. Note that we output to a
   * vector here because the size of these maps can get too large to save. The
   * max size of each map is equal to param max_output_map_size
   * @param max_output_map_size this function will convert all submap lidar
   * points to the world frame, but it's possible the map still gets too large,
   * so this will break it up
   * @param use_initial_world_frame set to true to use the initial world frame
   * from the local mapper, before global optimization
   * @param return vector of clouds
   */
  std::vector<PointCloud> GetLidarPointsInWorldFrame(
      int max_output_map_size, bool use_initial_world_frame = false) const;

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
   * attached sub-trajectories. Note that it is possible for a lidar keyframe
   * and camera keyframe to have the same timstamp but different poses, in that
   * case we use the camera keyrfame
   * @param return vectors of stamped poses, where poses are T_SUBMAP_FRAME
   */
  std::vector<Submap::PoseStamped> GetTrajectory() const;

  /**
   * @brief print relevant information about what is currently contained in this
   * submap. Example: pose, number of lidar scans and keypoints, etc...
   * @param stream input stream
   */
  void Print(std::ostream& stream = std::cout) const;

  /**
   * @brief save all contents of this submap. Format is as follows:
   *
   * output_dir/
   *    submap.json (general data)
   *    camera_model.json (intrinsics object)
   *    landmarks.json
   *    camera_keyframes.json
   *    /lidar_keyframes/
   *        /keyframe0/
   *            ...
   *        /keyframe1/
   *            ...
   *        ...
   *        /keyframeN/
   *            ...
   *    /subframes/
   *        subframe0.json
   *        subframe1.json
   *        ...
   *        subframeN.json
   *
   *    where lidar keyframe formats can be found in bs_common/ScanPose.h (see
   * SaveData function), camera_keyframes.json is a map from stamp (nsecs) to
   * pose vector (1 x 16)
   *
   * @param output_dir full path to output directory. This should be an empty
   * directory so as to not confuse with previous data
   */
  void SaveData(const std::string& output_dir);

  /**
   * @brief load submap data. This expects the same exact structure as
   * Save() function outputs to, as shown above. No other data can be in
   * this input directory.
   * @param input_dir full path to submap root directory
   * @param override_camera_model_pointer if set to false, this function will
   * not load the camera data from camera_model.json. The reason for this is
   * that it's likely we want all submaps to point to the same camera model
   * pointer, so we can construct the submaps with the same pointer, then not
   * override it when loading the data. The GlobalMap does this with its load
   * function.
   * @return true if successful
   */
  bool LoadData(const std::string& input_dir,
                bool override_camera_model_pointer);

 private:
  /**
   * @brief Get 3D positions of each landmark given current tracks and
   * camera poses. This fills in landmark_positions_
   * @param override_points if set to false, it will skip triangulation if
   * it has already been called
   */
  void TriangulateKeypoints(bool override_points = false);

  /**
   * @brief find the stored transform from submap to keyframe at time t. This
   * first looks in the camera keyframe poses, and if not found, it will look in
   * the lidar keyframes
   * @param time query time in nsecs
   * @param T_SUBMAP_KEYFRAME reference to result
   * @return true if successful, false otherwise
   */
  bool FindT_SUBMAP_KEYFRAME(uint64_t time,
                             Eigen::Matrix4d& T_SUBMAP_KEYFRAME) const;

  // general submap data
  ros::Time stamp_;
  int graph_updates_{0};
  fuse_variables::Position3DStamped position_;        // t_WORLD_SUBMAP
  fuse_variables::Orientation3DStamped orientation_;  // R_WORLD_SUBMAP
  std::shared_ptr<bs_common::ExtrinsicsLookupBase> extrinsics_;
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

  // subframe trajectory measurements, where poses are T_KEYFRAME_FRAME
  std::map<uint64_t, std::vector<PoseStamped>> subframe_poses_;

  // NOTE: all frames are baselink frames
};

}  // namespace global_mapping

}  // namespace bs_models