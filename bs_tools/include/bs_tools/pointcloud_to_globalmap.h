#pragma once

#include <bs_models/global_mapping/global_map.h>

namespace bs_tools {

/**
 * @brief TODO add description
 *
 */
class PointcloudToGlobalMap {
 public:
  /**
   * @brief struct to store all parameters needed for the this class, with
   * specified defaults and a method to override defaults with a json config
   * file.
   */
  struct Params {
    /** constructor to make sure special default variables are set */
    Params();

    // TODO: add all tunable params here. Any params that you don't want the
    // user to have control over, add as a private member to the
    // PointcloudToGlobalMap class

    /** Loads config settings from a json file. If config_path empty, it will
     * use default params defined herin. If config_path set to DEFAULT_PATH, it
     * will use the file in
     * beam_slam_launch/config/global_map/global_map_refinement.json */
    void LoadJson(const std::string& config_path);
  };

  /**
   * @brief delete default constructor
   */
  PointcloudToGlobalMap() = delete;

  /**
   * @brief constructor requiring a pointcloud, output path and optional params
   * object. This will load the params and then call CreateGlobalMap()
   * @param pointcloud full path to pointcloud file to be broken up into a
   * global map class. This can either be a .pcd or .ply
   * @param params see struct above
   */
  PointcloudToGlobalMap(const std::string& pointcloud,
                        const Params& params = Params());

  /**
   * @brief constructor requiring a pointcloud, output path and a path to a
   * params config json file. This constructor will call
   * params_.LoadJson(config_path), then call CreateGlobalMap()
   * @param pointcloud full path to pointcloud file to be broken up into a
   * global map class. This can either be a .pcd or .ply
   * @param config_path full path to json config file to load the params struct
   */
  PointcloudToGlobalMap(const std::string& pointcloud,
                        const std::string& config_path);

  /**
   * @brief default destructor
   */
  ~PointcloudToGlobalMap() = default;

  /**
   * @brief Store the global map data to an output directory so it can later be
   * loaded
   * @param output_path  full path to output path for global map data
   * @return true if successful, false otherwise.
   */
  bool SaveGlobalMapData(const std::string& output_path);

  /**
   * @brief Store the global map results to a form that is easy to visualize,
   * but cannot be re-loaded to a GlobalMap class.
   *
   * Note for Gabe: this function should be identical to:
   * GlobalMapRefinement.SaveResults()
   *
   * @param output_path  full path to output path for results
   * @return true if successful, false otherwise.
   */
  bool SaveGlobalMapResults(const std::string& output_path);

  /**
   * @brief return all the submaps from the global map that was generated
   * @return submaps that were generated from the input pointcloud when the
   * class was initialized
   */
  std::vector<std::shared_ptr<Submap>> GetSubmaps();

 private:
  /**
   * @brief This function takes the full pcd file and breaks it up into submaps
   * and then loads each submap into a GlobalMap object. The global map can then
   * be saved or returned using the functions below.
   * @return true if successful, false otherwise.
   */
  bool CreateGlobalMap();

  Params params_;
};

}  // namespace bs_tools