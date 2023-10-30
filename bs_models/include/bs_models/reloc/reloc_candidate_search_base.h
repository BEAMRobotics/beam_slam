#pragma once

#include <bs_models/global_mapping/submap.h>

namespace bs_models::reloc {

/**
 * @brief Reloc candidate search finds candidate relocs within submaps
 * and returns estimated relative poses
 */
class RelocCandidateSearchBase {
public:
  /**
   * @brief default constructor
   */
  RelocCandidateSearchBase() = default;

  /**
   * @brief default destructor
   */
  ~RelocCandidateSearchBase() = default;

  /**
   * @brief Pure virtual function that takes in a vector of submaps, a query
   * submap and finds candidate relocs with an estimated relative pose. The
   * candidates should be ordered based on the most likely candidate.
   * @param search_submaps vector of pointers to submaps
   * @param query_submap submap that we want to search for
   * @param matched_indices reference to vector of indices which represent the
   * candidate reloc submap indices
   * @param Ts_Candidate_Query reference to vector of transforms
   * from query submap to matched submap
   * @param ignore_last_n_submaps how many of the final submaps should be
   * ignored. This is useful when using this for loop closure where we don't
   * want to look in the last n submaps
   * @param use_initial_poses if set to true, we will use the initial pose in
   * the submaps. This is useful when running reloc on submaps that are being
   * optimized, but the initial query pose estimate is in the original world
   * frame
   */
  virtual void FindRelocCandidates(
      const std::vector<global_mapping::SubmapPtr>& search_submaps,
      const global_mapping::SubmapPtr& query_submap,
      std::vector<int>& matched_indices,
      std::vector<Eigen::Matrix4d, beam::AlignMat4d>& Ts_Candidate_Query,
      size_t ignore_last_n_submaps, bool use_initial_poses = false,
      const std::string& output_path = "") = 0;

  /**
   * @brief Factory method to create a object at runtime given a config file
   */
  static std::shared_ptr<RelocCandidateSearchBase>
      Create(const std::string& config_path);

protected:
};

} // namespace bs_models::reloc
