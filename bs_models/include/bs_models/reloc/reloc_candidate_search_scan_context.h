#pragma once

#include <optional>

#include <beam_matching/Matchers.h>

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
  struct MatchPair {
    uint64_t query_submap_scan_time;
    uint64_t matched_submap_scan_time;
    int matched_submap_id;
  };

  void LoadConfig();

  std::optional<Eigen::Matrix4d> AlignSubmapsFromScanMatches(
      const ScanPose& scan_pose_candidate, const ScanPose& scan_pose_query,
      const Eigen::Matrix4d& T_World_CandidateSubmap,
      const Eigen::Matrix4d& T_World_QuerySubmap) const;

  std::string config_path_;
  std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher_;
  std::unique_ptr<beam_matching::LoamMatcher> matcher_loam_;
  double submap_distance_threshold_m_;
};

} // namespace bs_models::reloc
