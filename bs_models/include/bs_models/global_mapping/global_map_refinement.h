#pragma once

#include <bs_models/global_mapping/global_map.h>
#include <bs_models/global_mapping/global_map_batch_optimization.h>
#include <bs_models/global_mapping/submap_alignment.h>
#include <bs_models/global_mapping/submap_pose_graph_optimization.h>
#include <bs_models/global_mapping/submap_refinement.h>

namespace bs_models::global_mapping {

/**
 * @brief This class is used to refine a global map's submaps.
 *
 * This is intended to be used by calling any of the 4 refinement steps, in any
 * order:
 *
 * (1) RunSubmapRefinement
 * (2) RunSubmapAlignment
 * (3) RunPoseGraphOptimization
 * (4) RunBatchOptimization
 *
 * An example ordering would be to run 1, 2, then 3. The idea behind this
 * ordering is that each submap refinement is independent of the other submaps,
 * but this might cause misalignment between them, so we then re-align them. The
 * PGO should then run better if the submaps have already been refined &
 * realigned. The submap refinement does take much longer than the PGO,
 * therefore the PGO may want to be run on its own, or vice-versa. We may want
 * to try different implementations of the submap refinement to optimize the
 * settings before running the PGO, in which case we might only want to refine
 * submaps. This interface allows the user to decide how they want to run the
 * map refinement.
 *
 * The batch optimization (4) is a replacement for running 1 to 4 which may work
 * better or worse
 *
 */
class GlobalMapRefinement {
public:
  struct Params {
    SubmapRefinement::Params submap_refinement;
    SubmapAlignment::Params submap_alignment;
    SubmapPoseGraphOptimization::Params submap_pgo;
    GlobalMapBatchOptimization::Params batch;

    /** Loads config settings from a json file. If config_path empty, it will
     * use default params defined herein. */
    void LoadJson(const std::string& config_path);
  };

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
   * @brief Two step batch optimization: 1. Go over all scans and register them
   * according to the registration method used. 2) find loops over entire
   * trajectory. Both measurements from 1 and 2 are optimized incrementally each
   * time we get to a loop
   * @param output_path save results to this path
   */
  bool RunBatchOptimization(const std::string& output_path = "");

  /**
   * @brief save results to a easily viewable format (i.e., pcd maps, poses,
   * ...)
   * @param output_path full path to output directory. Directory must exist
   * @param save_initial save results using initial estimate of poses stored
   * in the submaps.
   */
  void SaveResults(const std::string& output_path, bool save_initial = false);

  /**
   * @brief save global map data to a format that can be read later by the
   * global map object,
   * @param output_path full path to output directory. Directory must exist
   */
  void SaveGlobalMapData(const std::string& output_path);

private:
  Params params_;
  std::shared_ptr<GlobalMap> global_map_;
  Summary summary_;
};

} // namespace bs_models::global_mapping