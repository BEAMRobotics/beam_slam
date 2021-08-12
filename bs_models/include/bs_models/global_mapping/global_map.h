#pragma once

#include <fuse_core/transaction.h>

#include <beam_utils/pointclouds.h>

#include <bs_models/CameraMeasurementMsg.h>
#include <bs_models/LidarMeasurementMsg.h>
#include <bs_models/TrajectoryMeasurementMsg.h>
#include <bs_models/LandmarkMeasurementMsg.h>
#include <bs_common/extrinsics_lookup.h>
#include <bs_models/global_mapping/submap.h>
#include <bs_models/global_mapping/loop_closure/loop_closure_candidate_search_base.h>
#include <bs_models/global_mapping/loop_closure/loop_closure_refinement_base.h>

namespace bs_models {

namespace global_mapping {

/**
 * @brief This class takes care of all global mapping functionality. It received
 * incoming slam data from the local mapper, saves it into submaps and then runs
 * loop closure on the submaps to refine the final map.
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

    /** String describing the loop closure type to use.
     * Options:
     * - EUCDIST: Euclidean distance candidate search.
     */
    std::string loop_closure_candidate_search_type{"EUCDIST"};

    /** String describing the loop closure refinement type to use.
     * Options:
     * - ICP: ICP scan registration with lidar data
     * - GICP: GICP scan registration with lidar data
     * - NDT: NDT scan registration on lidar data
     * - LOAM: LOAM scan registration
     */
    std::string loop_closure_refinement_type{"ICP"};

    /** Full path to config file for loop closure candidate search. If blank, it
     * will use default parameters.*/
    std::string loop_closure_candidate_search_config{""};

    /** Full path to config file for loop closure refinement. If blank, it will
     * use default parameters.*/
    std::string loop_closure_refinement_config{""};

    /** covariance matrix from binary factors between scan poses which are added
     * from the local mapper results*/
    Eigen::Matrix<double, 6, 6> local_mapper_covariance;

    /** covariance matrix from binary factors between loop closures*/
    Eigen::Matrix<double, 6, 6> loop_closure_covariance;

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
   * @brief default constructor
   */
  GlobalMap() = default;

  /**
   * @brief constructor requiring only a pointer to camera model object
   * @param camera_model shared pointer to a camera model class
   */
  GlobalMap(const std::shared_ptr<beam_calibration::CameraModel>& camera_model);

  /**
   * @brief constructor that also takes a params struct.
   * @param camera_model shared pointer to a camera model class
   * @param params see struct above
   */
  GlobalMap(const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
            const Params& params);

  /**
   * @brief constructor that also takes a path to config file.
   * @param camera_model shared pointer to a camera model class
   * @param config_path full path to json config file
   */
  GlobalMap(const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
            const std::string& config_path);

  /**
   * @brief default destructor
   */
  ~GlobalMap() = default;

  /**
   * @brief get access to the submaps
   * @return pointer to vector of submaps stored in this global map
   */
  std::shared_ptr<std::vector<Submap>> GetSubmaps();

  /**
   * @brief set the submaps
   * @param submaps pointer to vector of submaps to be stored in this global map
   */
  void SetSubmaps(const std::shared_ptr<std::vector<Submap>>& submaps);  

  /**
   * @brief add a slam chunk measurement to the appropriate submap and returns a
   * transaction if a new submap is generated. This transaction will contain a
   * constraint between the new submap and the previous, and then initiate a
   * loop closure check on the previous submap to see if a loop closure
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
   * @brief takes the latest submap (back of vector) and adds a pose constraint
   * between it and it's previous submap. If it's the first submap, it will add
   * a perfect prior on this submap.
   * @return transaction which adds the new submap variables and a binary
   * constraints between the previous submap and the new one
   */
  fuse_core::Transaction::SharedPtr InitiateNewSubmapPose();

  /**
   * @brief Update submap poses with a new graph message
   * @param graph_msg updated graph which should have all submap poses stored
   */
  void UpdateSubmapPoses(fuse_core::Graph::ConstSharedPtr graph_msg);

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
   * initiating the loop closure pointer
   */
  void Setup();

  /**
   * @brief Get the appropriate submap id that a new measurement should be added
   * to. This will NOT create a new submap if the current measurement is
   * outside the current submap range (since we need the pose and stamp to
   * construct a submap). Note: since the incoming frame is still in world frame
   * of the local mapper, we need to use the initial T_WORLD_SUBMAP before any
   * loop closures were run.
   * @param T_WORLD_FRAME transform from current frame to local mapper's world
   * frame
   */
  int GetSubmapId(const Eigen::Matrix4d& T_WORLD_FRAME);

  /**
   * @brief Get loop closure measurements by comparing the second last submap to
   * all previous submaps. The reason we compare the second last submap is
   * because we don't want to find loop closures until the submap is complete,
   * this ensures a best estimate of the loop closure constraint.
   * @return fuse transaction with the frame to frame constraints between two
   * loop closure poses
   */
  fuse_core::Transaction::SharedPtr FindLoopClosures();

  Params params_;
  std::shared_ptr<std::vector<Submap>> submaps_;

  bs_common::ExtrinsicsLookup& extrinsics_ =
      bs_common::ExtrinsicsLookup::GetInstance();

  std::shared_ptr<beam_calibration::CameraModel> camera_model_;

  std::unique_ptr<LoopClosureCandidateSearchBase>
      loop_closure_candidate_search_;
  std::unique_ptr<LoopClosureRefinementBase> loop_closure_refinement_;

  // params only tunable here
  int max_output_map_size_{1000000};
};

}  // namespace global_mapping

}  // namespace bs_models