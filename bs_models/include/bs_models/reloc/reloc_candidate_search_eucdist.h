#pragma once

#include <map>

#include <ros/time.h>

#include <beam_utils/pointclouds.h>
#include <bs_models/reloc/reloc_candidate_search_base.h>

namespace bs_models::reloc {

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
   * @brief See base class
   */
  void FindRelocCandidates(
      const std::vector<global_mapping::SubmapPtr>& search_submaps,
      const global_mapping::SubmapPtr& query_submap,
      std::vector<int>& matched_indices,
      std::vector<Eigen::Matrix4d, beam::AlignMat4d>& Ts_Candidate_Query,
      size_t ignore_last_n_submaps, bool use_initial_poses = false) override;

private:
  /**
   * @brief Method for loading a config json file.
   */
  void LoadConfig() override;

  double distance_threshold_m_{5};
};

} // namespace bs_models::reloc
