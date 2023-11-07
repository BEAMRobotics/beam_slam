#pragma once

#include <pcl/kdtree/kdtree_flann.h>

#include <beam_matching/Matchers.h>

#include <bs_models/global_mapping/global_map.h>
#include <bs_models/reloc/reloc_candidate_search_base.h>
#include <bs_models/reloc/reloc_refinement_base.h>
#include <bs_models/scan_registration/multi_scan_registration.h>
#include <bs_models/scan_registration/scan_to_map_registration.h>

namespace sr = bs_models::scan_registration;

namespace bs_models::global_mapping {

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
  struct LoopClosureParams {
    /** Full path to config file for loop closure candidate search. If blank, it
     * will use default parameters.*/
    std::string candidate_search_config;

    /** Full path to config file for loop closure refinement. If blank, it will
     * use default parameters.*/
    std::string refinement_config;

    /** Weights to assign to loop closure measurements */
    Eigen::Matrix<double, 6, 6> loop_closure_covariance;

    /** Weights to assign to local mapper measurements */
    Eigen::Matrix<double, 6, 6> local_mapper_covariance;
  };

  struct SubmapRefinementParams {
    /** Full path to config file for scan registration. If blank, it will use
     * default parameters.*/
    std::string scan_registration_config;

    /** Full path to config file for matcher. If blank, it will use default
     * parameters.*/
    std::string matcher_config;
  };

  struct SubmapAlignmentParams {
    /** Full path to config file for matcher. If blank, it will use default
     * parameters.*/
    std::string matcher_config;
  };

  struct Params {
    LoopClosureParams loop_closure;
    SubmapRefinementParams submap_refinement;
    SubmapAlignmentParams submap_alignment;

    /** Loads config settings from a json file. If config_path empty, it will
     * use default params defined herein. */
    void LoadJson(const std::string& config_path);
  };

  struct RegistrationResult {
    RegistrationResult(const Eigen::Matrix4d& Ti, const Eigen::Matrix4d& Tf);
    double dt; // mm
    double dR; // deg
  };
  using RegistrationResults = std::map<ros::Time, RegistrationResult>;

  struct Summary {
    RegistrationResults submap_refinement;
    RegistrationResults submap_alignment;

    void Save(const std::string& output_path) const;
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
   * @brief Calls RefineSubmap on all submaps, with a check to make sure each
   * one passed, otherwise it exits
   * @param output_path save results to this path
   * @return true if successful
   */
  bool RunSubmapRefinement(const std::string& output_path = "");

  /**
   * @brief Calls AlignSubmaps on all contiguous submap pairs, with a check to
   * make sure each one passed, otherwise it exits
   * @param output_path save results to this path
   * @return true if successful
   */
  bool RunSubmapAlignment(const std::string& output_path = "");

  /**
   * @brief calls loop closure on the submaps using the global map
   * @param output_path save results to this path
   */
  bool RunPoseGraphOptimization(const std::string& output_path = "");

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
  bool RefineSubmap(SubmapPtr& submap, const std::string& output_path);

  /**
   * @brief Aligns the tgt submap to the reference submap. This is designed to
   * be called one by one starting with the second submap. Since drift builds up
   * over time, we want to always use the initial relative poses between the two
   * submaps to initialize the registration.
   * @param submap_ref const reference to submap to be aligned to
   * @param submap_tgt reference to submap to be aligned
   * @return true if successful
   */
  bool AlignSubmaps(const SubmapPtr& submap_ref, SubmapPtr& submap_tgt,
                    const std::string& output_path);

  /**
   * @brief setup general things needed when class is instatiated, such as
   * initiating the loop closure pointer
   */
  void Setup();

  Params params_;
  std::shared_ptr<GlobalMap> global_map_;

  // PGO:
  std::shared_ptr<reloc::RelocCandidateSearchBase>
      loop_closure_candidate_search_;
  std::shared_ptr<reloc::RelocRefinementBase> loop_closure_refinement_;

  // Submap alignment
  std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher_;
  std::unique_ptr<beam_matching::Matcher<beam_matching::LoamPointCloudPtr>>
      matcher_loam_;

  Summary summary_;

  // params only tunable here
  int pgo_skip_first_n_submaps_{3};
  double pose_prior_noise_{1e-9};
};

} // namespace bs_models::global_mapping