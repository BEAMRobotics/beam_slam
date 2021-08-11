#pragma once

#include <fuse_core/transaction.h>
#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/constraint.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_graphs/hash_graph.h>

#include <beam_utils/pointclouds.h>
#include <beam_matching/Matchers.h>

#include <bs_models/global_mapping/submap.h>
#include <bs_models/global_mapping/loop_closure/loop_closure_candidate_search_base.h>
#include <bs_models/global_mapping/loop_closure/loop_closure_refinement_base.h>
#include <bs_models/frame_to_frame/scan_registration/multi_scan_registration.h>

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
    frame_to_frame::MultiScanLoamRegistration::Params scan_reg_params;

    /** loam scan matcher params */
    beam_matching::LoamParams loam_matcher_params;

    /** Loads config settings from a json file. If config_path empty, it will
     * use default params defined herin. If config_path set to DEFAULT_PATH, it
     * will use the file in
     * beam_slam_launch/config/global_map/global_map_refinement.json */
    void LoadJson(const std::string& config_path);

    /** Save contents of struct to a json which can be loaded using LoadJson()
     */
    void SaveJson(const std::string& filename);
  };

  /**
   * @brief delete default constructor
   */
  GlobalMapRefinement() = delete;

  /**
   * @brief constructor requiring only a pointer to camera model object
   * @param submaps reference to a vector of submaps to refine
   */
  GlobalMapRefinement(std::vector<Submap>& submaps);

  /**
   * @brief constructor that also takes a params struct.
   * @param submaps reference to a vector of submaps to refine
   * @param params see struct above
   */
  GlobalMapRefinement(std::vector<Submap>& submaps, const Params& params);

  /**
   * @brief constructor that also takes a path to config file.
   * @param submaps reference to a vector of submaps to refine
   * @param config_path full path to json config file
   */
  GlobalMapRefinement(std::vector<Submap>& submaps,
                      const std::string& config_path);

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

 private:
  /**
   * @brief Refines a single submap
   * @param submap reference to submap to be refined
   * @return true if successful
   */
  bool RefineSubmap(Submap& submap);

  /**
   * @brief setup general things needed when class is instatiated, such as
   * initiating the loop closure pointer
   */
  void Setup();

  Params params_;
  std::vector<Submap>& submaps_;

  // PGO:
  std::unique_ptr<LoopClosureCandidateSearchBase>
      loop_closure_candidate_search_;
  std::unique_ptr<LoopClosureRefinementBase> loop_closure_refinement_;
};

}  // namespace global_mapping

}  // namespace bs_models