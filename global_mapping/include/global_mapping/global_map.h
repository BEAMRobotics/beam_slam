#pragma once

#include <fuse_core/transaction.h>

#include <beam_utils/pointclouds.h>

#include <global_mapping/submap.h>
#include <global_mapping/CameraMeasurement.h>
#include <global_mapping/LidarMeasurement.h>
#include <global_mapping/TrajectoryMeasurement.h>
#include <global_mapping/LandmarkMeasurement.h>
// #include <global_mapping/loop_closure.h>

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
     * - EUCDISTICP: Euclidean distance candidate search plus ICP scan
     * registration
     * - EUCDISTGICP: Euclidean distance candidate search plus GICP scan
     * registration
     * - EUCDISTNDT: Euclidean distance candidate search plus NDT scan
     * registration
     * - EUCDISTLOAM: Euclidean distance candidate search plus LOAM scan
     * registration
     */
    std::string loop_closure_type{"EUCDISTICP"};

    /** Full path to config file for loop closure. If blank, it will use default
     * parameters.*/
    std::string loop_closure_config{""};

    /** Loads config settings from a json file. */
    void LoadJson(const std::string& config_path);
  };

  /**
   * @brief constructor that takes an optional path to config file.
   * @param config_path full path to json config file
   */
  GlobalMap(const std::string& config_path = "");

  /**
   * @brief default destructor
   */
  ~GlobalMap() = default;

  /**
   * @brief add a camera measurement to the appropriate submap
   * @param measurement camera measurement to add
   */
  void AddCameraMeasurement(const CameraMeasurement& measurement);

  /**
   * @brief add a lidar measurement to the appropriate submap
   * @param measurement lidar measurement to add
   */
  void AddLidarMeasurement(const LidarMeasurement& measurement);

  /**
   * @brief add a trajectory measurement to the appropriate submap. This is
   * usually used to add a trajectory coming from higher rate sensore to fill
   * the gap between lower rate sensors. Example: camera keyframes and lidar
   * keyframes will be stored in the submap, but to get high rate pose estimates
   * between them, we may want to add relative poses from the IMU
   * (preintegration)
   * @param measurement trajectory measurement to add
   */
  void AddTrajectoryMeasurement(const TrajectoryMeasurement& measurement);

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
  void SaveTrajectoryFile(const std::string& output_path);

  /**
   * @brief saves the trajectory as a pointcloud
   * @param output_path where to save the trajectory
   */
  void SaveTrajectoryCloud(const std::string& output_path);
  

 private:
  /**
   * @brief initiates the loop closure pointer
   */
  void InitiateLoopClosure();

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
  //   std::unique_ptr<LoopClosureBase> loop_closure_;
};

}  // namespace global_mapping