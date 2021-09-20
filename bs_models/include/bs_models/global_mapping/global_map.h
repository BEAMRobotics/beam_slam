#pragma once

#include <queue>

#include <fuse_core/transaction.h>
#include <sensor_msgs/PointCloud2.h>

#include <beam_utils/pointclouds.h>
#include <beam_filtering/Utils.h>

#include <bs_common/bs_msgs.h>
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_models/global_mapping/submap.h>
#include <bs_models/reloc/reloc_candidate_search_base.h>
#include <bs_models/reloc/reloc_refinement_base.h>

namespace bs_models {

namespace global_mapping {

/**
 * @brief Enum class for the different types of ROS maps to be published
 */
enum class RosMapType {
  LIDARSUBMAP = 0,
  LIDARGLOBALMAP,
  VISUALSUBMAP,
  VISUALGLOBALMAP,
  LIDARNEW,
};

using RosMap = std::pair<RosMapType, sensor_msgs::PointCloud2>;

/**
 * @brief Enum class for storing types of submaps
 */
enum class SubmapType {
  NONE = 0,
  OFFLINE,
  ONLINE,
};

/**
 * @brief This class takes care of all global mapping functionality. It received
 * incoming slam data from the local mapper, saves it into submaps and then runs
 * reloc on the submaps to refine the final map.
 */
class GlobalMap {
 public:
  /**
   * @brief struct to store all parameters needed for the global map, with
   * specified defaults and a method to override defaults with a json config
   * file.
   */
  struct Params {
    /** constructor to make sure covariances are set */
    Params();

    /** Max linear distance between poses in a submap */
    double submap_size{10};

    /** String describing the reloc type to use.
     * Options:
     * - EUCDIST: Euclidean distance candidate search.
     */
    std::string reloc_candidate_search_type{"EUCDIST"};

    /** String describing the reloc refinement type to use.
     * Options:
     * - ICP: ICP scan registration with lidar data
     * - GICP: GICP scan registration with lidar data
     * - NDT: NDT scan registration on lidar data
     * - LOAM: LOAM scan registration
     */
    std::string reloc_refinement_type{"ICP"};

    /** Full path to config file for reloc candidate search. If blank, it
     * will use default parameters.*/
    std::string reloc_candidate_search_config{""};

    /** Full path to config file for reloc refinement. If blank, it will
     * use default parameters.*/
    std::string reloc_refinement_config{""};

    /** covariance matrix from binary factors between scan poses which are added
     * from the local mapper results*/
    Eigen::Matrix<double, 6, 6> local_mapper_covariance;

    /** covariance matrix from binary factors between relocs*/
    Eigen::Matrix<double, 6, 6> reloc_covariance;

    /* Output filters to apply to lidar submaps before adding them to the Ros
     * maps list */
    std::vector<beam_filtering::FilterParamsType> ros_submap_filter_params;

    /* Output filters to apply to global lidar maps before adding them to the
     * Ros maps list. Note: submap filters are also applied before while
     * creating the global map */
    std::vector<beam_filtering::FilterParamsType> ros_globalmap_filter_params;

    /** Loads config settings from a json file. If config_path empty, it will
     * use default params defined herin. If config_path set to DEFAULT_PATH, it
     * will use the file in
     * beam_slam_launch/config/global_map/global_map.json */
    void LoadJson(const std::string& config_path);

    /** Save contents of struct to a json which can be loaded using LoadJson()
     */
    void SaveJson(const std::string& filename);
  };

  /**
   * @brief delete default constructor
   */
  GlobalMap() = delete;

  /**
   * @brief constructor that requires a root directory to global map data. For
   * data format, see SaveData function
   * @param data_root_directory full path to global map data.
   */
  GlobalMap(const std::string& data_root_directory);

  /**
   * @brief constructor requiring only a pointer to camera model object
   * @param camera_model shared pointer to a camera model class
   * @param extrinsics pointer to extrinsics
   */
  GlobalMap(const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
            const std::shared_ptr<bs_common::ExtrinsicsLookupBase>& extrinsics);

  /**
   * @brief constructor that also takes a params struct.
   * @param camera_model shared pointer to a camera model class
   * @param extrinsics pointer to extrinsics
   * @param params see struct above
   */
  GlobalMap(const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
            const std::shared_ptr<bs_common::ExtrinsicsLookupBase>& extrinsics,
            const Params& params);

  /**
   * @brief constructor that also takes a path to config file.
   * @param camera_model shared pointer to a camera model class
   * @param extrinsics pointer to extrinsics
   * @param config_path full path to json config file
   */
  GlobalMap(const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
            const std::shared_ptr<bs_common::ExtrinsicsLookupBase>& extrinsics,
            const std::string& config_path);

  /**
   * @brief default destructor
   */
  ~GlobalMap() = default;

  /**
   * @brief get access to the online submaps (see variable below for definition)
   * @return vector of pointers to the online submaps stored in this global map
   */
  std::vector<std::shared_ptr<Submap>> GetOnlineSubmaps();

  /**
   * @brief get access to the offline submaps (see variable below for
   * definition)
   * @return vector of pointers to the offline submaps stored in this global map
   */
  std::vector<std::shared_ptr<Submap>> GetOfflineSubmaps();

  /**
   * @brief set online the submaps vector
   * @param submaps vector of pointers to submaps to be stored in this global
   * map
   */
  void SetOnlineSubmaps(std::vector<std::shared_ptr<Submap>>& submaps);

  /**
   * @brief set offline the submaps vector
   * @param submaps vector of pointers to submaps to be stored in this global
   * map
   */
  void SetOfflineSubmaps(std::vector<std::shared_ptr<Submap>>& submaps);

  /**
   * @brief Sets store_newly_completed_submaps_ param. See description below for
   * what his does.
   * @param store_new_submaps
   */
  void SetStoreNewSubmaps(bool store_new_submaps);

  /**
   * @brief Sets store_new_data_ param. See description below for
   * what his does.
   * @param store_new_scans
   */
  void SetStoreNewScans(bool store_new_scans);

  /**
   * @brief Sets store_updated_global_map param. See description below for what
   * his does.
   * @param store_updated_global_map
   */
  void SetStoreUpdatedGlobalMap(bool store_updated_global_map);

  /**
   * @brief If param store_newly_completed_submaps_ is set to true, the global
   * map will store newly completed submaps as a PointCloud2 ROS messages. It
   * will do this for lidar points and camera keypoints. If the param
   * store_updated_global_map_ is set to true, it will do the same for all map
   * points whenever the graph is updated. All these pointclouds will be added
   * to a vector of pointers to ROS maps which will be retrieved and cleared
   * from this function.
   * @return vector of pointers to ROS maps
   */
  std::vector<std::shared_ptr<RosMap>> GetRosMaps();

  /**
   * @brief add a slam chunk measurement to the appropriate submap and returns a
   * transaction if a new submap is generated. This transaction will contain a
   * constraint between the new submap and the previous, and then initiate a
   * reloc check on the previous submap to see if a reloc
   * constraints can also be added to the transaction.
   * @param cam_measurement camera measurement to add
   * @param lid_measurement lidar measurement to add
   * @param traj_measurement trajectory measurement to add
   * @param T_WORLD_BASELINK baselink pose of slam chunk message. World frame
   * here is the local map's world frame. This function will convert the poses
   * to relative transforms instead of absolute.
   * @param stamp stamp associated with the baselink pose
   *
   * NOTE: All data should
   * be in baselink_frame_ already
   */
  fuse_core::Transaction::SharedPtr AddMeasurement(
      const CameraMeasurementMsg& cam_measurement,
      const LidarMeasurementMsg& lid_measurement,
      const TrajectoryMeasurementMsg& traj_measurement,
      const Eigen::Matrix4d& T_WORLD_BASELINK, const ros::Time& stamp);

  /**
   * @brief This function takes a reloc request message and tries to determine
   * if the reloc request pose is inside an existing submap.
   *
   * There are process works as follows:
   *
   * (1) run reloc candidacy search on all offline maps if they exist.
   *  - if none returned, then skip to 2
   *  - check that candidate submaps are not the current active submap
   *  - run reloc refinement on all candidate submaps
   *  - store the best refined pose and submap index (if at least one refinement
   *    was successful)
   *
   * (2) 1 was unsuccessful, run reloc candidacy search on all online
   *     maps
   *  - if non returned, then we are done
   *  - check that candidate submaps are not the current active submap
   *  - run reloc refinement on all candidate submaps
   *  - store the best refined pose and submap index (if at least one refinement
   *    was successful)
   *
   * (3) If 1 or 2 successful, create a SubmapMsg with the resulting submap and
   *     refined pose within that submap. If unsuccessful, return nothing
   *
   * @param reloc_request_msg input reloc information
   * @param submap_msg reference to submap message to fill should this be
   * successful
   * @return true if a new submap was found and reloc refinement was successful
   */
  bool ProcessRelocRequest(const bs_common::RelocRequestMsg& reloc_request_msg,
                           bs_common::SubmapMsg& submap_msg);

  /**
   * @brief Update submap poses with a new graph message
   * @param graph_msg updated graph which should have all submap poses stored
   * @param update_time ros time of this update
   */
  void UpdateSubmapPoses(fuse_core::Graph::ConstSharedPtr graph_msg,
                         const ros::Time& update_time = ros::Time(0));

  /**
   * @brief Calling this will trigger a loop closure (reloc) search for the last
   * submap. This is convenient for completing a mapping session where the final
   * submap won't be complete so we wouldn't have run reloc on it.
   * @return transaction with new variables and constraints if applicable
   */
  fuse_core::Transaction::SharedPtr TriggerLoopClosure();

  /**
   * @brief Save full global map to a format that can be reloaded later for new
   * mapping sessions. Format will be as follows:
   *
   *  /output_path/
   *    params.json
   *    camera_model.json
   *    /submap0/
   *      ...
   *    /submap1/
   *      ...
   *    ...
   *    /submapN/
   *       ...
   *
   *  Where the format of the submap data is described in submap.h
   *
   * @param output_path full path to directory
   */
  void SaveData(const std::string& output_path);

  /**
   * @brief load all global map data from a previous mapping session. This
   * requires the exact format show above in SaveFullGlobalMap() function. There
   * must be no other data in this directory
   * @param root_directory root directory of the global map data
   * @return true if successful
   */
  bool Load(const std::string& root_directory);

  /**
   * @brief Save each lidar submap to pcd files. A lidar submap consists of an
   * aggregation of all scans in the submap transformed to the world frame using
   * the submap pose estimate and the relative pose measurements of all scans
   * relative to their submap anchor.
   * @param output_path where to save the submaps
   * @param save_initial set to true to save the initial map from the
   * local mapper, before global optimization
   */
  void SaveLidarSubmaps(const std::string& output_path,
                        bool save_initial = false);

  /**
   * @brief Save each keypoint submap to pcd files. A keypoint submap contains
   * all estimated 3D keypoint locations which have been transformed to the worl
   * frame using the submap pose estimate (anchor).
   * @param output_path where to save the submaps
   * @param save_initial set to true to save the initial map from the
   * local mapper, before global optimization
   */
  void SaveKeypointSubmaps(const std::string& output_path,
                           bool save_initial = false);

  /**
   * @brief saves the trajectory as a posefile
   * @param output_path where to save the trajectory
   * @param save_initial set to true to save the initial trajectory from the
   * local mapper, before global optimization
   */
  void SaveTrajectoryFile(const std::string& output_path,
                          bool save_initial = true);

  /**
   * @brief saves the trajectory as a pointcloud
   * @param output_path where to save the trajectory
   * @param save_initial set to true to save the initial trajectory from the
   * local mapper, before global optimization
   */
  void SaveTrajectoryClouds(const std::string& output_path,
                            bool save_initial = true);

  /**
   * @brief saves the pose of each submap as a pointcloud with RGB frames
   * @param output_path where to save the pointcloud
   * @param save_initial set to true to save the initial trajectory from the
   * local mapper, before global optimization
   */
  void SaveSubmapFrames(const std::string& output_path,
                        bool save_initial = true);

 private:
  /**
   * @brief setup general things needed when class is instatiated, such as
   * initiating the reloc pointer
   */
  void Setup();

  /**
   * @brief Get the appropriate submap id that a new measurement should be added
   * to. This will NOT create a new submap if the current measurement is
   * outside the current submap range (since we need the pose and stamp to
   * construct a submap). Note: since the incoming frame is still in world frame
   * of the local mapper, we need to use the initial T_WORLD_SUBMAP before any
   * relocs were run.
   * @param T_WORLD_FRAME transform from current frame to local mapper's world
   * frame
   */
  int GetSubmapId(const Eigen::Matrix4d& T_WORLD_FRAME);

  /**
   * @brief takes the latest submap (back of vector) and adds a pose constraint
   * between it and it's previous submap. If it's the first submap, it will add
   * a perfect prior on this submap.
   * @return transaction which adds the new submap variables and a binary
   * constraints between the previous submap and the new one
   */
  fuse_core::Transaction::SharedPtr InitiateNewSubmapPose();

  /**
   * @brief Get loop closure measurements by comparing the a submap to
   * all previous submaps. By default, every new submap will trigger a loop
   * closure for the second last submap. The reason we compare the second last
   * submap is because we don't want to find relocs until the submap is
   * complete, this ensures a best estimate of the reloc constraint. When
   * a global map is complete, you should trigger reloc against the last
   * submap which probably still isn't complete. Note that loop closure uses
   * reloc under the hood to find the candidate submaps and refined poses
   * @param query_index index of submap to look for relocs against
   * @return fuse transaction with the frame to frame constraints between two
   * reloc poses
   */
  fuse_core::Transaction::SharedPtr RunLoopClosure(int query_index);

  /**
   * @brief adds submap points (lidar points and camera keypoints) to the queue
   * of ros messages to be published
   * @param submap_id index to submap to add
   */
  void AddRosSubmap(int submap_id);

  /**
   * @brief adds global map points (lidar points and camera keypoints) to the
   * vector of ros messages to be published
   */
  void AddRosGlobalMap();

  /**
   * @brief adds new lidar scans  to the
   * queue of ros messages to be published
   * @param cloud cloud to add
   * @param T_WORLD_BASELINK pose of scan
   * @param stamp
   */
  void AddNewRosScan(const PointCloud& cloud,
                     const Eigen::Matrix4d& T_WORLD_BASELINK,
                     const ros::Time& stamp);

  /**
   * @brief Fill a submap message for publishing.
   * @param submap_msg reference to submap to fill
   * @param lidar_points lidar points to add (in any frame)
   * @param loam_points loam pointcloud to add (in any frame)
   * @param keypoints 3D keypoint locations (in any frame) to add
   * @param descriptors vector of descriptors to add, these must be the same
   * order as their corresponding keypoints
   * @param descriptor_type
   * @param T transform to apply to all points (lidar_points, loam_points,
   * keypoints) to get them into the local mapper's world frame
   */
  void FillSubmapMsg(SubmapMsg& submap_msg, const PointCloud& lidar_points,
                     const beam_matching::LoamPointCloud& loam_points,
                     const PointCloud& keypoints,
                     const std::vector<std::vector<float>>& descriptors,
                     uint8_t descriptor_type, const Eigen::Matrix4d& T) const;

  /**
   * @brief convert a vector of floats to a pcl pointcloud xyz. This also checks
   * that the number of points is divisible by 3 to make sure the correct format
   * is used, otherwise it returns an empty cloud
   * @param points points to add to cloud
   * @return pointcloud
   */
  PointCloud RosCloudToPclCloud(const std::vector<float>& points);

  Params params_;

  /** If set to true, this will store recently completed submaps as a
   * PointCloud2 message so that it can be extracted using
   * GetRosMaps() function and published over ROS. NOTE: this cannot
   * be set by or saved to a config file. It must be set by calling
   * SetSaveNewlyCompletedSubmaps(). The reason for this is that we want to set
   * this from the sensor model, and make sure it isn't overriden by a config
   * file.*/
  bool store_newly_completed_submaps_{false};

  /** If set to true, this will store the recently updated global map to a
   * PointCloud2 message so that it can be extracted using
   * GetRosMaps() function and published over ROS. NOTE: this cannot
   * be set by or saved to a config file. It must be set by calling
   * SetSaveUpdatedGlobalMap(). The reason for this is that we want to set this
   * from the sensor model, and make sure it isn't overriden by a config file.*/
  bool store_updated_global_map_{false};

  /** If set to true, this will store new lidar data to a
   * PointCloud2 message so that it can be extracted using
   * GetRosMaps() function and published over ROS. NOTE: We do not have the
   * ability to publish new camera keypoints because the global map doesn't
   * actually store 3D positions unless asked to triangulate. NOTE: this cannot
   * be set by or saved to a config file. It must be set by calling
   * SetSaveNewData(). The reason for this is that we want to set this
   * from the sensor model, and make sure it isn't overriden by a config file.*/
  bool store_new_scans_{false};

  std::shared_ptr<bs_common::ExtrinsicsLookupBase> extrinsics_;
  std::shared_ptr<beam_calibration::CameraModel> camera_model_;

  /** online submaps are the submaps that are being build when AddMeasurement is
   * called, or when a previous global map is loaded in the constructor. It also
   * has the set and get functions: SetOnlineSubmaps(), GetOnlineSubmaps()
   */
  std::vector<std::shared_ptr<Submap>> online_submaps_;

  /** Offline submaps can only be preloaded using SetOfflineSubmaps(). This
   * vector of offline submaps represents some pre-loaded global map which will
   * be used during RelocRequest to see if the robot is currently within an
   * offline submap which will then get sent back to the local mapper. If it is
   * determined that the robot is not within any offline submap, then it will
   * search within the online submaps.
   */
  std::vector<std::shared_ptr<Submap>> offline_submaps_;

  std::unique_ptr<reloc::RelocCandidateSearchBase> reloc_candidate_search_;
  std::unique_ptr<reloc::RelocRefinementBase> reloc_refinement_;

  /** These params are used for tracking what submap are we currently in and
   * other required information */
  Eigen::Matrix4d T_WORLDLM_WORLDOFF_{Eigen::Matrix4d::Identity()};
  bool T_WORLDLM_WORLDOFF_found_{false};
  int active_submap_id_{0};
  SubmapType active_submap_type_{SubmapType::NONE};

  // ros maps
  std::queue<std::shared_ptr<RosMap>> ros_submaps_;
  std::queue<std::shared_ptr<RosMap>> ros_new_scans_;
  std::shared_ptr<RosMap> ros_global_lidar_map_;
  std::shared_ptr<RosMap> ros_global_keypoints_map_;

  // -------------------------
  // params only tunable here:
  // -------------------------
  /** when saving lidar submap points, it will break the cloud into clouds of
   * max size defined by this param. This avoids files that are very large. */
  int max_output_map_size_ = 10e6;

  /** prior noise to add to first submap pose */
  double pose_prior_noise_{1e-9};

  /** maximum number of ros maps to store at a time. Note that we will only keep
   * one global map at a time, but many submaps can be kept */
  int max_num_ros_submaps_{10};

  /** maximum number of new scans to store at a time.  */
  int max_num_new_scans_{50};

  /** keep track of graph updates from ROS */
  int global_map_updates_{0};
  int new_scans_counter_{0};
  ros::Time last_update_time_;
};

}  // namespace global_mapping

}  // namespace bs_models