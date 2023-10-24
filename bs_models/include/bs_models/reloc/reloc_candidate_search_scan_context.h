#pragma once

#include <bs_models/reloc/reloc_candidate_search_base.h>

namespace bs_models::reloc {

class RelocCandidateSearchScanContext : public RelocCandidateSearchBase {
public:
  /**
   * @brief constructor taking in a file path to a json config file
   */
  RelocCandidateSearchScanContext(const std::string& config);

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
  void LoadConfig();
  std::string config_path_;
};

} // namespace bs_models::reloc
