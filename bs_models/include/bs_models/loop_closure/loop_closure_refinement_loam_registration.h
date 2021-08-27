#pragma once

#include <map>

#include <ros/time.h>
#include <fuse_core/transaction.h>
#include <nlohmann/json.hpp>

#include <beam_utils/pointclouds.h>
#include <beam_matching/Matchers.h>
#include <bs_models/loop_closure/loop_closure_refinement_base.h>
#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>

namespace bs_models {

namespace loop_closure {

using namespace beam_matching;
using namespace global_mapping;

/**
 * @brief Loop closure refinement with loam scan matching
 */
class LoopClosureRefinementLoam : public LoopClosureRefinementBase {
 public:
  // Inherit base class constructors
  using LoopClosureRefinementBase::LoopClosureRefinementBase;

  /**
   * @brief another constructor that takes in parameters
   * @param matcher_config full path to matcher config json
   */
  LoopClosureRefinementLoam(
      const Eigen::Matrix<double, 6, 6>& loop_closure_covariance,
      const std::string& matcher_config = "");

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
      const std::shared_ptr<Submap>& matched_submap,
      const std::shared_ptr<Submap>& query_submap,
      const Eigen::Matrix4d& T_MATCH_QUERY_EST) override;

 private:
  /**
   * @brief method for loading a config json file.
   */
  void LoadConfig() override;

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
  bool GetRefinedT_MATCH_QUERY(const std::shared_ptr<Submap>& matched_submap,
                               const std::shared_ptr<Submap>& query_submap,
                               const Eigen::Matrix4d& T_MATCH_QUERY_EST,
                               Eigen::Matrix4d& T_MATCH_QUERY_OPT);

  std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher_;
  std::string matcher_config_{""};
  bool covariance_set_{false};
  Eigen::Matrix<double, 6, 6> loop_closure_covariance_;
};

}  // namespace loop_closure

}  // namespace bs_models
