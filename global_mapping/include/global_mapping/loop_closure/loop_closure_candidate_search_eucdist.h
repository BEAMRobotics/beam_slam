#pragma once

#include <map>

#include <ros/time.h>

#include <beam_utils/pointclouds.h>
#include <global_mapping/loop_closure/loop_closure_candidate_search_base.h>

namespace global_mapping {

/**
 * @brief
 */
class LoopClosureCandidateSearchEucDist
    : public LoopClosureCandidateSearchBase {
 public:
  // Inherit base class constructors
  using LoopClosureCandidateSearchBase::LoopClosureCandidateSearchBase;

  /**
   * @brief Pure virtual function that takes in a vector of submaps, a query
   * index and finds candidate loop closures with an estimated relative pose
   * @param submaps vector of submaps
   * @param current_index index of submap to check for loops
   * @param matched_indices reference to vector of indices which represent the
   * candidate loop closure submap indices
   * @param estimated_poses reference to vector of transforms from query submap
   * to matched submap (T_MATCH_QUERY)
   */
  void FindLoopClosureCandidates(
      const std::vector<Submap>& submaps, int current_index,
      std::vector<int>& matched_indices,
      std::vector<Eigen::Matrix4d, pose_allocator>& estimated_poses) override;

 private:
  /**
   * @brief pure virtual method for loading a config json file.
   */
  void LoadConfig() override;
};

}  // namespace global_mapping