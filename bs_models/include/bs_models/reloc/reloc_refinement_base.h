#pragma once

#include <map>

#include <fuse_core/transaction.h>
#include <ros/time.h>

#include <beam_utils/pointclouds.h>
#include <bs_models/global_mapping/submap.h>

namespace bs_models::reloc {

struct RelocRefinementResults {
  Eigen::Matrix4d T_MATCH_QUERY{Eigen::Matrix4d::Identity()};
  std::optional<Eigen::Matrix<double, 6, 6>> covariance;
  bool successful{false};
};

/**
 * @brief A reloc refinement step that takes an estimated pose
 * from the candidate search and refines the relative pose between the two
 * candidate locations.
 */
class RelocRefinementBase {
public:
  /**
   * @brief constructor with a required path to a json config
   * @param config path to json config file. If empty, it will use default
   * parameters
   */
  RelocRefinementBase(const std::string& config) : config_path_(config){};

  /**
   * @brief default destructor
   */
  ~RelocRefinementBase() = default;

  /**
   * @brief Pure virtual function that runs the refinement between two
   * candidate reloc submaps.
   * @param matched_submap
   * @param query_submap
   * @param T_MATCH_QUERY_EST estimated transform between match and query
   * submaps. This usually comes from the RelocCandidateSearch class
   * @param output_path optional output path. If not empty, it will output
   * results to this folder. If non empty but doesn't exist, throw exception.
   * Results should be saved in folders named by the query_submap timestamp
   */
  virtual RelocRefinementResults
      RunRefinement(const global_mapping::SubmapPtr& matched_submap,
                    const global_mapping::SubmapPtr& query_submap,
                    const Eigen::Matrix4d& T_MATCH_QUERY_EST,
                    const std::string& output_path = "") = 0;

  /**
   * @brief Factory method to create a object at runtime given a config file
   */
  static std::shared_ptr<RelocRefinementBase>
      Create(const std::string& config_path);

protected:
  std::string config_path_;
};

} // namespace bs_models::reloc
