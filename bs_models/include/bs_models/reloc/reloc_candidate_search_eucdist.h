#pragma once

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
   * @brief constructor taking in a file path to a json config file
   */
  RelocCandidateSearchEucDist(const std::string& config);

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
      size_t ignore_last_n_submaps, const std::string& output_path) override;

private:
  void LoadConfig();
  std::string config_path_;
  double distance_threshold_m_{5};
};

} // namespace bs_models::reloc
