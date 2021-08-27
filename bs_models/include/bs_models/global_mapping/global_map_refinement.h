#pragma once

#include <bs_models/global_mapping/global_map.h>
#include <bs_models/loop_closure/loop_closure_candidate_search_base.h>
#include <bs_models/loop_closure/loop_closure_refinement_base.h>
#include <bs_models/scan_registration/scan_to_map_registration.h>
#include <bs_models/scan_registration/multi_scan_registration.h>

namespace sr = bs_models::scan_registration;

namespace bs_models {
namespace global_mapping {

/**
 * @brief This class is used to refine a global map's submaps.
 *
 * This is intended to be used in 3 steps:
 *
 * (1) Construct the object and supply the list of submaps (or global map)
 * along with parameters
 *
 * (2) Call RunSubmapRefinement to optimize the data in
 * each submap
 *
 * (3) Call RunPoseGraphOptimization (PGO) to refine the pose of the
 * submaps relative to each other
 *
 * The idea behind this ordering is that each submap refinement is independent
 * of the other submaps, while the PGO is performed between all submaps. The PGO
 * should run better if the submaps have already been refined. The submap
 * refinement does take much longer than the PGO, therefore the PGO may want to
 * be run on its own, or vice-versa. We may want to try different
 * implementations of the submap refinement to optimize the settings before
 * running the PGO, in which case we might only want to refine submaps. This
 * interface allows the user to decide how they want to run the map refinement.
 *
 */
class GlobalMapRefinement {
 public:
  /**
   * @brief struct to store all parameters needed for the global map, with
   * specified defaults and a method to override defaults with a json config
   * file.
   */
  struct Params {
    /** constructor to make sure special default variables are set */
    Params();

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

    /** String describing the type of scan registrattion to use
     * Options:
     * - MULTISCAN
     * - SCANTOMAP
     */
    std::string scan_registration_type{"SCANTOMAP"};

    /** Full path to config file for loop closure candidate search. If blank, it
     * will use default parameters.*/
    std::string loop_closure_candidate_search_config{""};

    /** Full path to config file for loop closure refinement. If blank, it will
     * use default parameters.*/
    std::string loop_closure_refinement_config{""};

    /** covariance matrix for binary factors between scan registration
     * measurements during submap refinement*/
    Eigen::Matrix<double, 6, 6> scan_reg_covariance;

    /** covariance matrix from binary factors between loop closures*/
    Eigen::Matrix<double, 6, 6> loop_closure_covariance;

    /** multi scan registration params */
    sr::MultiScanLoamRegistration::Params multi_scan_reg_params;

    /** scan to map registration params */
    sr::ScanToMapLoamRegistration::Params scan_to_map_reg_params;

    /** loam scan matcher params */
    beam_matching::LoamParams loam_matcher_params;

    /** Loads config settings from a json file. If config_path empty, it will
     * use default params defined herin. If config_path set to DEFAULT_PATH, it
     * will use the file in
     * beam_slam_launch/config/global_map/global_map_refinement.json */
    void LoadJson(const std::string& config_path);
  };

  /**
   * @brief delete default constructor
   */
  GlobalMapRefinement() = delete;

  /**
   * @brief constructor requiring a path to a directory containing global map
   * data, and optional params struct.
   * @param global_map_data_dir directory must contain data from only one global
   * map. See GlobalMap.h (SaveData function) for format of data
   * @param params see struct above
   */
  GlobalMapRefinement(const std::string& global_map_data_dir,
                      const Params& params = Params());

  /**
   * @brief constructor requiring a path to a directory containing global map
   * data, and optional path to a global map refinement config file.
   * @param global_map_data_dir directory must contain data from only one global
   * map. See GlobalMap.h (SaveData function) for format of data
   * @param params see struct above
   */
  GlobalMapRefinement(const std::string& global_map_data_dir,
                      const std::string& config_path = "");

  /**
   * @brief constructor requiring a list of submaps and optional params struct.
   * @param submaps  vector of pointers to submaps to refine
   * @param params see struct above
   */
  GlobalMapRefinement(std::shared_ptr<GlobalMap>& global_map,
                      const Params& params = Params());

  /**
   * @brief constructor requiring a list of submaps and optional path to config
   * file.
   * @param submaps pointer to a vector of submaps to refine
   * @param config_path full path to json config file
   */
  GlobalMapRefinement(std::shared_ptr<GlobalMap>& global_map,
                      const std::string& config_path = "");

  /**
   * @brief default destructor
   */
  ~GlobalMapRefinement() = default;

  /**
   * @brief Calls RefineSUbmap on all submaps, with a check to make sure each
   * one passed, otherwise it exits
   * @return true if successful
   */
  bool RunSubmapRefinement();

  /**
   * @brief TODO
   */
  bool RunPoseGraphOptimization();

  /**
   * @brief save results to a easily viewable format (i.e., pcd maps, poses,
   * ...)
   * @param output_path full path to output directory. Directory must exist
   * @param save_initial save results using initial estimate of poses stored in
   * the submaps.
   */
  void SaveResults(const std::string& output_path, bool save_initial = false);

  /**
   * @brief save global map data to a format that can be read later by the
   * global map object,
   * @param output_path full path to output directory. Directory must exist
   */
  void SaveGlobalMapData(const std::string& output_path);

 private:
  /**
   * @brief Refines a single submap
   * @param submap reference to submap to be refined
   * @return true if successful
   */
  bool RefineSubmap(std::shared_ptr<Submap>& submap);

  /**
   * @brief setup general things needed when class is instatiated, such as
   * initiating the loop closure pointer
   */
  void Setup();

  Params params_;
  std::shared_ptr<GlobalMap> global_map_;
  std::vector<std::shared_ptr<Submap>> submaps_;

  // PGO:
  std::unique_ptr<loop_closure::LoopClosureCandidateSearchBase>
      loop_closure_candidate_search_;
  std::unique_ptr<loop_closure::LoopClosureRefinementBase>
      loop_closure_refinement_;
};

}  // namespace global_mapping

}  // namespace bs_models