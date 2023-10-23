#pragma once

#include <map>

#include <fuse_core/transaction.h>
#include <nlohmann/json.hpp>
#include <ros/time.h>

#include <beam_matching/LoamMatcher.h>
#include <beam_utils/pointclouds.h>
#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>
#include <bs_models/reloc/reloc_refinement_base.h>

namespace bs_models { namespace reloc {

/**
 * @brief reloc refinement with loam scan matching
 */
class RelocRefinementLoam : public RelocRefinementBase {
public:
  /**
   * @brief Constructor that only takes in a config path and covariance matrix
   * @param config full path to config json
   * @param reloc_covariance
   */
  RelocRefinementLoam(const std::string& config);

  /**
   * @brief See base class
   */
  RelocRefinementResults
      RunRefinement(const global_mapping::SubmapPtr& matched_submap,
                    const global_mapping::SubmapPtr& query_submap,
                    const Eigen::Matrix4d& T_MATCH_QUERY_EST,
                    const std::string& output_path = "") override;

private:
  /**
   * @brief method for loading a config json file.
   */
  void LoadConfig();

  /**
   * @param takes all params loaded from LoadConfig, and initializes all member
   * classes/variables
   */
  void Setup();

  /**
   * @brief Calculate a refined pose between submaps using loam registration
   * @param submap_cloud submap cloud in submap frame
   * @param query_cloud either a scan to register against a submap, or a query
   * submap cloud to match against some other submap. This is in the query frame
   * submaps
   * @param T_SUBMAP_QUERY_EST estimated transform between the query cloud and
   * the submap. This usually is determined with a class derived from
   * RelocCandidateSearchBase
   * @param output_path will save results if not empty
   * @param T_SUBMAP_QUERY_OPT reference to the resulting refined transform from
   * query scan to the submap in question
   * @param covariance this is calculated by loam
   */
  bool GetRefinedT_SUBMAP_QUERY(
      const beam_matching::LoamPointCloud& submap_cloud,
      const beam_matching::LoamPointCloud& query_cloud,
      const Eigen::Matrix4d& T_SUBMAP_QUERY_EST, const std::string& output_path,
      Eigen::Matrix4d& T_SUBMAP_QUERY_OPT,
      Eigen::Matrix<double, 6, 6>& covariance);

  std::string matcher_config_;
  std::unique_ptr<beam_matching::LoamMatcher> matcher_;
  std::string source_{"RelocRefinementLoam"};
};

}} // namespace bs_models::reloc
