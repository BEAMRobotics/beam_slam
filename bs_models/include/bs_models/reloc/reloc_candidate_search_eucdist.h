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
 * previous submaps and calculates the norm between the query submap and the
 * candidate reloc submap. If the norm is below some threshold, then the
 * candidate is return along with the relative pose (from current submap pose
 * estimates) between the two.
 */
class RelocCandidateSearchEucDist
    : public RelocCandidateSearchBase {
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
   * query index and finds candidate relocs with an estimated relative
   * pose. It does this by iterating from the first submap, to two submaps
   * before the query submap (we don't want adjacent submaps) and checks the
   * euclidean distance between the submaps. If the distance is less than
   * distance_threshold_m_ param, then it is a candidate reloc.
   * @param submaps vector of pointers to submaps
   * @param query_index index of submap to check for loops
   * @param matched_indices reference to vector of indices which represent the
   * candidate reloc submap indices
   * @param estimated_poses reference to vector of transforms from query submap
   * to matched submap (T_MATCH_QUERY)
   */
  void FindRelocCandidates(
      const std::vector<std::shared_ptr<Submap>>& submaps, int query_index,
      std::vector<int>& matched_indices,
      std::vector<Eigen::Matrix4d, pose_allocator>& estimated_poses) override;

 private:
  /**
   * @brief Method for loading a config json file.
   */
  void LoadConfig() override;

  double distance_threshold_m_{5};
};

}  // namespace reloc

}  // namespace bs_models
