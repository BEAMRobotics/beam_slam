#pragma once

#include <fuse_graphs/hash_graph.h>

#include <beam_matching/Matcher.h>

#include <bs_models/global_mapping/utils.h>

namespace bs_models::global_mapping {

class GlobalMapBatchOptimization {
public:
  struct Params {
    std::string scan_registration_config;
    std::string matcher_config;
    bool update_graph_on_all_scans;
    double lc_dist_thresh_m;
    double lc_min_traj_dist_m;
    double lc_scan_context_dist_thres;
  };

  GlobalMapBatchOptimization() = delete;

  GlobalMapBatchOptimization(const Params& params,
                             const std::string& output_path = "");

  ~GlobalMapBatchOptimization() = default;

  bool Run(std::vector<SubmapPtr> submaps);

private:
  void RunLoopClosureOnAllScans(ScanPose& query);

  pcl::PointCloud<pcl::PointXYZI> AggregateScan(uint64_t scan_time_ns) const;

  Params params_;
  std::string output_path_;

  std::vector<SubmapPtr> submaps_;
  std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher_;
  std::unique_ptr<beam_matching::Matcher<beam_matching::LoamPointCloudPtr>>
      matcher_loam_;
  std::shared_ptr<fuse_graphs::HashGraph> graph_;
  std::map<uint64_t, int> scan_stamp_to_submap_id_;

  // params only tunable here:
  int scans_to_aggregate_{30};
  const Eigen::Vector3f scan_context_voxel_filter_{0.04, 0.04, 0.04};
};

} // namespace bs_models::global_mapping