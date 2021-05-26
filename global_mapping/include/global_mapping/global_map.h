#pragma once

#include <fuse_core/transaction.h>

#include <beam_utils/pointclouds.h>

#include <global_mapping/submap.h>
#include <global_mapping/CameraMeasurementMsg.h>
#include <global_mapping/LidarMeasurementMsg.h>
#include <global_mapping/TrajectoryMeasurementMsg.h>
#include <global_mapping/LandmarkMeasurementMsg.h>
#include <beam_common/extrinsics_lookup.h>
#include <global_mapping/loop_closure/loop_closure_candidate_search_base.h>
#include <global_mapping/loop_closure/loop_closure_refinement_base.h>

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

    /** Full path to config file for loop closure candidate search. If blank, it will use default
     * parameters.*/
    std::string loop_closure_candidate_search_config{""};

    /** Full path to config file for loop closure refinement. If blank, it will use default
     * parameters.*/
    std::string loop_closure_refinement_config{""};

    /** Loads config settings from a json file. */
    void LoadJson(const std::string& config_path);
  };

  /**
   * @brief constructor requiring only a pointer to an extrinsics lookup object
   * @param extrinsics object for looking up extrinsics
   */
  GlobalMap(const std::shared_ptr<ExtrinsicsLookup>& extrinsics);

  /**
   * @brief constructor that also takes a params struct.
   * @param extrinsics object for looking up extrinsics
   * @param params see struct above
   */
  GlobalMap(const std::shared_ptr<ExtrinsicsLookup>& extrinsics,
            const Params& params);

  /**
   * @brief constructor that also takes a path to config file.
   * @param extrinsics object for looking up extrinsics
   * @param config_path full path to json config file
   */
  GlobalMap(const std::shared_ptr<ExtrinsicsLookup>& extrinsics,
            const std::string& config_path);

  /**
   * @brief default destructor
   */
  ~GlobalMap() = default;

  /**
   * @brief add a camera measurement to the appropriate submap
   * @param measurement camera measurement to add.
   * NOTE: All data should be in baselink_frame_ already
   */
  void AddCameraMeasurement(const CameraMeasurementMsg& measurement);

  /**
   * @brief add a lidar measurement to the appropriate submap
   * @param measurement lidar measurement to add
   * NOTE: All data should be in baselink_frame_ already
   */
  void AddLidarMeasurement(const LidarMeasurementMsg& measurement);

  /**
   * @brief add a trajectory measurement to the appropriate submap. This is
   * usually used to add a trajectory coming from higher rate sensore to fill
   * the gap between lower rate sensors. Example: camera keyframes and lidar
   * keyframes will be stored in the submap, but to get high rate pose estimates
   * between them, we may want to add relative poses from the IMU
   * (preintegration)
   * @param measurement trajectory measurement to add
   * NOTE: All transforms should be from baselink_frame_ to world_frame_
   */
  void AddTrajectoryMeasurement(const TrajectoryMeasurementMsg& measurement);

  /**
   * @brief Get loop closure measurements by comparing the current submap to all
   * previous submaps
   * @return fuse transaction with the frame to frame constraints between two
   * loop closure poses
   */
  fuse_core::Transaction::SharedPtr FindLoopClosures();

  /**
   * @brief Update submap poses with a new graph message
   * @param graph_msg updated graph which should have all submap poses stored
   */
  void UpdateSubmapPoses(fuse_core::Graph::ConstSharedPtr graph_msg);

  /**
   * @brief Save each lidar submap to pcd files. A lidar submap consists of an
   * aggregation of all scans in the submap transformed to the worl frame using
   * the submap pose estimate and the relative pose measurements of all scans
   * relative to their submap anchor.
   * @param output_path where to save the submaps
   */
  void SaveLidarSubmaps(const std::string& output_path);

  /**
   * @brief Save each keypoint submap to pcd files. A keypoint submap contains
   * all estimated 3D keypoint locations which have been transformed to the worl
   * frame using the submap pose estimate (anchor).
   * @param output_path where to save the submaps
   */
  void SaveKeypointSubmaps(const std::string& output_path);

  /**
   * @brief Save a map containing all lidar submaps expressed in the world frame
   * @param output_path where to save the map
   */
  void SaveFullLidarMap(const std::string& output_path);

  /**
   * @brief Save a map containing all submap keypoints expressed in the world
   * frame
   * @param output_path where to save the map
   */
  void SaveFullKeypointMap(const std::string& output_path);

  /**
   * @brief saves the trajectory as a posefile
   * @param output_path where to save the trajectory
   */
  void SaveTrajectoryFiles(const std::string& output_path);

  /**
   * @brief saves the trajectory as a pointcloud
   * @param output_path where to save the trajectory
   */
  void SaveTrajectoryClouds(const std::string& output_path);

  /**
   * @brief saves the pose of each submap as a pointcloud with RGB frames
   * @param output_path where to save the pointcloud
   */
  void SaveSubmapFrames(const std::string& output_path);

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
   */
  int GetSubmapId(const Eigen::Matrix4d& T_WORLD_FRAME);

  /**
   * @brief Converts an array of float containing a pose measurement [R | t] (of
   * size 3 x 4) to a 4 x 4 Eigen Matrix
   */
  Eigen::Matrix4d VectorToEigenTransform(const std::vector<float>& v);

  Params params_;
  std::vector<Submap> submaps_;
  std::shared_ptr<ExtrinsicsLookup> extrinsics_;
  std::unique_ptr<LoopClosureCandidateSearchBase> loop_closure_candidate_search_;
  std::unique_ptr<LoopClosureRefinementBase> loop_closure_refinement_;

  // All poses will be stored w.r.t to these frame. By default, baselink is set
  // to camera frame stored in extrinsics_ and world frame is set to "world".
  // See Setup()
  std::string baselink_frame_;
  std::string world_frame_;
};

}  // namespace global_mapping