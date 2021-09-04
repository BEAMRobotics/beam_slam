#pragma once

#include <map>

#include <ros/time.h>
#include <fuse_core/transaction.h>
#include <nlohmann/json.hpp>

#include <beam_utils/pointclouds.h>
#include <beam_matching/LoamMatcher.h>
#include <bs_models/loop_closure/loop_closure_refinement_base.h>
#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>

namespace bs_models {

namespace loop_closure {

/**
 * @brief Loop closure refinement with loam scan matching
 */
class LoopClosureRefinementLoam : public LoopClosureRefinementBase {
 public:
  /**
   * @brief Constructor that only takes in a config path and covariance matrix
   * @param config full path to config json
   * @param loop_closure_covariance
   */
  LoopClosureRefinementLoam(
      const Eigen::Matrix<double, 6, 6>& loop_closure_covariance,
      const std::string& config = "");

  /**
   * @brief Generate a fuse transaction between two candidate loop closure
   * submaps
   * @param matched_submap submap that a new query submap matches to
   * @param query_submap new submap that we are adding constraints with previous
   * submaps
   * @param T_MATCH_QUERY_EST estimated transform between the two submaps. This
   * is determined with a class derived from LoopClosureCandidateSearchBase
   */
  fuse_core::Transaction::SharedPtr GenerateTransaction(
      const std::shared_ptr<global_mapping::Submap>& matched_submap,
      const std::shared_ptr<global_mapping::Submap>& query_submap,
      const Eigen::Matrix4d& T_MATCH_QUERY_EST) override;

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
   * @brief Calculate a refined pose between submaps using scan registration
   * @param matched_submap submap that a new query submap matches to
   * @param query_submap new submap that we are adding constraints with previous
   * submaps
   * @param T_MATCH_QUERY_EST estimated transform between the two submaps. This
   * is determined with a class derived from LoopClosureCandidateSearchBase
   * @param T_MATCH_QUERY_OPT reference to the resulting refined transform from
   * query submap to matched submap
   */
  bool GetRefinedT_MATCH_QUERY(
      const std::shared_ptr<global_mapping::Submap>& matched_submap,
      const std::shared_ptr<global_mapping::Submap>& query_submap,
      const Eigen::Matrix4d& T_MATCH_QUERY_EST,
      Eigen::Matrix4d& T_MATCH_QUERY_OPT);

  std::string matcher_config_;
  std::unique_ptr<beam_matching::Matcher<beam_matching::LoamPointCloudPtr>>
      matcher_;
  Eigen::Matrix<double, 6, 6> loop_closure_covariance_;
  std::string source_{"LOAMSCANREGLOOPCLOSURE"};
};

}  // namespace loop_closure

}  // namespace bs_models
