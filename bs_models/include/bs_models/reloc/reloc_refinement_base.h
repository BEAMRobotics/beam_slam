#pragma once

#include <map>

#include <ros/time.h>
#include <fuse_core/transaction.h>

#include <beam_utils/pointclouds.h>
#include <bs_models/global_mapping/submap.h>

namespace bs_models {

namespace reloc {

/**
 * @brief A reloc refinement step that takes an estimated pose
 * from the candidate search and refines the relative pose between the two
 * candidate locations
 */
class RelocRefinementBase {
 public:
  /**
   * @brief constructor with an optional path to a json config
   * @param config path to json config file. If empty, it will use default
   * parameters
   */
  RelocRefinementBase(const std::string& config = "")
      : config_path_(config){};

  /**
   * @brief default destructor
   */
  ~RelocRefinementBase() = default;

  /**
   * @brief Pure virtual function that generate a fuse transaction between two
   * candidate reloc submaps
   * @param matched_submap
   * @param query_submap
   */
  virtual fuse_core::Transaction::SharedPtr GenerateTransaction(
      const std::shared_ptr<global_mapping::Submap>& matched_submap,
      const std::shared_ptr<global_mapping::Submap>& query_submap,
      const Eigen::Matrix4d& T_MATCH_QUERY_EST) = 0;

 protected:
  std::string config_path_;

  /* Debugging tools that can only be set here */
  bool output_results_{false};
  std::string debug_output_path_{"/home/nick/results/beam_slam/reloc/"};
  std::string output_path_stamped_; // to be created in implementation
};

}  // namespace reloc

}  // namespace bs_models
