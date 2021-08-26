#pragma once

#include <map>

#include <ros/time.h>

#include <beam_utils/pointclouds.h>
#include <bs_models/global_mapping/submap.h>

namespace bs_models {

namespace global_mapping {

/**
 * @brief Loop candidate search finds candidate loop closures between submaps
 * and returns estimated relative poses
 */
class LoopClosureCandidateSearchBase {
 public:
  /**
   * @brief constructor with an optional path to a json config
   * @param config path to json config file. If empty, it will use default
   * parameters
   */
  LoopClosureCandidateSearchBase(const std::string& config = "")
      : config_path_(config) {}

  /**
   * @brief default destructor
   */
  ~LoopClosureCandidateSearchBase() = default;

  /**
   * @brief Pure virtual function that takes in a vector of submaps, a query
   * index and finds candidate loop closures with an estimated relative pose
   * @param submaps vector of pointers to submaps
   * @param query_index index of submap to check for loops
   * @param matched_indices reference to vector of indices which represent the
   * candidate loop closure submap indices
   * @param estimated_poses reference to vector of transforms from query submap
   * to matched submap (T_MATCH_QUERY)
   */
  virtual void FindLoopClosureCandidates(
      const std::vector<std::shared_ptr<Submap>>& submaps, int query_index,
      std::vector<int>& matched_indices,
      std::vector<Eigen::Matrix4d, pose_allocator>& estimated_poses) = 0;

 protected:
  /**
   * @brief pure virtual method for loading a config json file.
   */
  virtual void LoadConfig() = 0;

  std::string config_path_;
};

}  // namespace global_mapping

}  // namespace bs_models
