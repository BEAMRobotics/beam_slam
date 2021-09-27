#pragma once

#include <map>

#include <ros/time.h>

#include <beam_utils/pointclouds.h>
#include <bs_models/reloc/reloc_candidate_search_base.h>

namespace bs_models {

namespace reloc {

using namespace global_mapping;

/**
 * @brief This class implements a reloc candidate search class. To
 * look for candidate relocs, this class simply looks through all
 * submaps supplied and calculates the norm between the candidate submap pose
 * and the query pose. If the norm is below some threshold, then the candidate
 * is return along with the relative pose between the two.
 */
class RelocCandidateSearchEucDist : public RelocCandidateSearchBase {
 public:
  /**
   * @brief constructor that takes in config path
   * @param config_path full path to config json
   */
  RelocCandidateSearchEucDist(const std::string& config_path);

  /**
   * @brief another constructor that takes in parameters
   * @param distance_threshold_m submaps closer than this will be considered as
   * reloc candidates
   */
  RelocCandidateSearchEucDist(double distance_threshold_m);

  /**
   * @brief Overrides the virtual function that takes in a vector of submaps, a
   * query pose and finds candidate relocs with an estimated relative pose. The
   * candidates should be ordered based on the most likely candidate.
   * @param submaps vector of pointers to submaps
   * @param T_WORLD_QUERY we look for submaps that contains this pose (note
   * query pose is the pose of the baselink)
   * @param matched_indices reference to vector of indices which represent the
   * candidate reloc submap indices
   * @param estimated_poses reference to vector of transforms from query pose
   * to matched submap (T_SUBMAPCANDIDATE_QUERY)
   * @param ignore_last_n_submaps how many of the final submaps should be
   * ignored. This is useful when using this for loop closure where we don't
   * want to look in the last n submaps
   * @param use_initial_poses if set to true, we will use the initial pose in
   * the submaps. This is useful when running reloc on submaps that are being
   * optimized, but the initial query pose estimate is in the original world
   * frame
   */
  void FindRelocCandidates(
      const std::vector<SubmapPtr>& submaps,
      const Eigen::Matrix4d& T_WORLD_QUERY, std::vector<int>& matched_indices,
      std::vector<Eigen::Matrix4d, pose_allocator>& estimated_poses,
      size_t ignore_last_n_submaps = 0,
      bool use_initial_poses = false) override;

 private:
  /**
   * @brief Method for loading a config json file.
   */
  void LoadConfig() override;

  double distance_threshold_m_{5};
};

}  // namespace reloc

}  // namespace bs_models
