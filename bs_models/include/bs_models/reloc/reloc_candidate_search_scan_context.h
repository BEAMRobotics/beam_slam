#pragma once

#include <optional>

#include <beam_filtering/Utils.h>
#include <beam_matching/Matchers.h>

#include <bs_models/reloc/reloc_candidate_search_base.h>

namespace bs_models::reloc {

using PointCloudSC = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudSCPtr = std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>;

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
      size_t ignore_last_n_submaps, bool use_initial_poses,
      const std::string& output_path) override;

private:
  struct MatchPair {
    int candidate_submap_id;
    int candidate_scan_id;
    PointCloudSCPtr candidate_scan;
    int query_scan_id;
  };

  void LoadConfig();

  std::optional<Eigen::Matrix4d> AlignSubmapsFromScanMatches(
      const PointCloudSCPtr& scan_candidate, const PointCloudSCPtr& scan_query,
      const Eigen::Matrix4d& T_SubmapCandidate_Lidar,
      const Eigen::Matrix4d& T_SubmapQuery_Lidar,
      const Eigen::Matrix4d& T_World_CandidateSubmap,
      const Eigen::Matrix4d& T_World_QuerySubmap,
      const std::string& output_path) const;

  std::vector<PointCloudSCPtr>
      AggregateSubmapScans(const global_mapping::SubmapPtr& submap) const;

  std::vector<uint64_t>
      GetTimesToAggregate(const std::map<uint64_t, ScanPose>& keyframes,
                          int center_id) const;
  std::string config_path_;
  std::vector<beam_filtering::FilterParamsType> filters_;
  std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher_;
  double submap_distance_threshold_m_;
  double scan_context_dist_thres_{0.3};
  int num_scans_to_aggregate_{20};
};

} // namespace bs_models::reloc
