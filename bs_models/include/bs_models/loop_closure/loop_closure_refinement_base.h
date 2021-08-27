#pragma once

#include <map>

#include <ros/time.h>
#include <fuse_core/transaction.h>

#include <beam_utils/pointclouds.h>
#include <bs_models/global_mapping/submap.h>

namespace bs_models {

namespace loop_closure {

using namespace global_mapping;    

/**
 * @brief A loop closure refinement step that takes an estimated pose
 * from the candidate search and refines the relative pose between the two
 * candidate locations
 */
class LoopClosureRefinementBase {
 public:
  /**
   * @brief constructor with an optional path to a json config
   * @param config path to json config file. If empty, it will use default
   * parameters
   */
  LoopClosureRefinementBase(const std::string& config = "")
      : config_path_(config){};

  /**
   * @brief default destructor
   */
  ~LoopClosureRefinementBase() = default;

  /**
   * @brief Pure virtual function that generate a fuse transaction between two
   * candidate loop closure submaps
   * @param matched_submap
   * @param query_submap
   */
  virtual fuse_core::Transaction::SharedPtr GenerateTransaction(
      const std::shared_ptr<Submap>& matched_submap,
      const std::shared_ptr<Submap>& query_submap,
      const Eigen::Matrix4d& T_MATCH_QUERY_EST) = 0;

 protected:
  /**
   * @brief pure virtual method for loading a config json file.
   */
  virtual void LoadConfig() = 0;

  std::string config_path_;
};

}  // namespace loop_closure

}  // namespace bs_models