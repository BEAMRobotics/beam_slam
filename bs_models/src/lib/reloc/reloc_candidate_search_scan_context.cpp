#include <bs_models/reloc/reloc_candidate_search_scan_context.h>

#include <filesystem>

#include <nlohmann/json.hpp>
#include <pcl/common/transforms.h>

#include <beam_matching/Scancontext.h>
#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <bs_common/utils.h>

namespace bs_models::reloc {

using namespace beam_matching;

RelocCandidateSearchScanContext::RelocCandidateSearchScanContext(
    const std::string& config)
    : config_path_(config) {
  LoadConfig();
}

void RelocCandidateSearchScanContext::LoadConfig() {
  if (config_path_.empty()) {
    BEAM_INFO("No config file provided to RelocCandidateSearchEucDist, using "
              "default parameters.");
    return;
  }

  nlohmann::json J;
  BEAM_INFO("Loading reloc config: {}", config_path_);
  if (!beam::ReadJson(config_path_, J)) {
    BEAM_ERROR("Unable to read config");
    throw std::runtime_error{"Unable to read config"};
  }

  beam::ValidateJsonKeysOrThrow(
      std::vector<std::string>{"type", "submap_distance_threshold_m",
                               "matcher_config", "scan_context_dist_thres",
                               "num_scans_to_aggregate", "filters"},
      J);
  std::string type = J["type"];
  if (type != "SCANCONTEXT") {
    BEAM_ERROR(
        "Invalid config file provided to RelocCandidateSearchScanContext: {}",
        type);
    throw std::runtime_error{"invalid config file"};
  }

  submap_distance_threshold_m_ = J["submap_distance_threshold_m"];
  scan_context_dist_thres_ = J["scan_context_dist_thres"];
  num_scans_to_aggregate_ = J["num_scans_to_aggregate"];
  filters_ = beam_filtering::LoadFilterParamsVector(J["filters"]);

  std::string matcher_config_rel = J["matcher_config"];
  if (matcher_config_rel.empty()) {
    BEAM_ERROR("Reloc candidate search cannot have an empty matcher_config");
    throw std::runtime_error{"invalid json inputs"};
  }
  std::string matcher_config = beam::CombinePaths(
      bs_common::GetBeamSlamConfigPath(), matcher_config_rel);

  // get type of matcher
  auto matcher_type = GetTypeFromConfig(matcher_config);
  if (matcher_type == MatcherType::LOAM) {
    BEAM_ERROR("detected LOAM matcher config type. This isn't supported.");
    throw std::runtime_error{"unsupported matcher type"};
  } else if (matcher_type == MatcherType::ICP) {
    matcher_ = std::make_unique<IcpMatcher>(IcpMatcher::Params(matcher_config));
  } else if (matcher_type == MatcherType::GICP) {
    matcher_ =
        std::make_unique<GicpMatcher>(GicpMatcher::Params(matcher_config));
  } else if (matcher_type == MatcherType::NDT) {
    matcher_ = std::make_unique<NdtMatcher>(NdtMatcher::Params(matcher_config));
  } else {
    BEAM_ERROR("Invalid matcher type");
    throw std::invalid_argument{"invalid json"};
  }
}

void RelocCandidateSearchScanContext::FindRelocCandidates(
    const std::vector<global_mapping::SubmapPtr>& search_submaps,
    const global_mapping::SubmapPtr& query_submap,
    std::vector<int>& matched_indices,
    std::vector<Eigen::Matrix4d, beam::AlignMat4d>& Ts_Candidate_Query,
    size_t ignore_last_n_submaps, bool use_initial_poses,
    const std::string& output_path) {
  matched_indices.clear();
  Ts_Candidate_Query.clear();
  if (search_submaps.size() <= ignore_last_n_submaps) { return; }

  // first, get submap ids that are closer than submap distance threshold
  std::map<double, int> initial_candidates_sorted;
  for (int i = 0; i < search_submaps.size() - ignore_last_n_submaps; i++) {
    Eigen::Matrix4d T_World_SubmapCandidate =
        use_initial_poses ? search_submaps.at(i)->T_WORLD_SUBMAP_INIT()
                          : search_submaps.at(i)->T_WORLD_SUBMAP();
    Eigen::Matrix4d T_World_SubmapQuery =
        use_initial_poses ? query_submap->T_WORLD_SUBMAP_INIT()
                          : query_submap->T_WORLD_SUBMAP();
    Eigen::Matrix4d T_SubmapCandidate_SubmapQuery =
        beam::InvertTransform(T_World_SubmapCandidate) * T_World_SubmapQuery;
    double distance = T_SubmapCandidate_SubmapQuery.block(0, 3, 3, 1).norm();

    if (distance < submap_distance_threshold_m_) {
      initial_candidates_sorted.emplace(distance, i);
    }
  }

  if (initial_candidates_sorted.empty()) { return; }

  // aggregate scans in query submap
  std::vector<PointCloudSCPtr> query_submap_scans =
      AggregateSubmapScans(query_submap);

  // build map from scan context scores to match pairs for all candidate submaps
  std::map<float, MatchPair> sc_dist_to_match_pair;
  for (const auto& [distance, submap_id] : initial_candidates_sorted) {
    SCManager sc_manager;
    sc_manager.SC_DIST_THRES = scan_context_dist_thres_;

    // iterate through scans in candidate submap and build a scan context
    // database
    std::vector<PointCloudSCPtr> candidate_submap_scans =
        AggregateSubmapScans(search_submaps.at(submap_id));
    for (const auto& candidate_scan : candidate_submap_scans) {
      sc_manager.makeAndSaveScancontextAndKeys(*candidate_scan);
    }

    // iterate through scans in query submap, and find best match in candidate
    // submap
    for (int query_scan_id = 0; query_scan_id < query_submap_scans.size();
         query_scan_id++) {
      const auto& query_scan = query_submap_scans.at(query_scan_id);
      std::pair<int, float> candidate_scan_id_and_dist =
          sc_manager.detectLoopClosureID(*query_scan, 0);
      if (candidate_scan_id_and_dist.first == -1) { continue; }
      MatchPair match_pair;
      match_pair.candidate_submap_id = submap_id;
      match_pair.candidate_scan_id = candidate_scan_id_and_dist.first;
      match_pair.query_scan_id = query_scan_id;
      match_pair.candidate_scan =
          candidate_submap_scans.at(candidate_scan_id_and_dist.first);
      sc_dist_to_match_pair.emplace(candidate_scan_id_and_dist.second,
                                    match_pair);
    }
  }

  // iterate through all match pair, starting with lowest SC distance and try to
  // align. If successful then return, if not then try next match pair
  for (auto iter = sc_dist_to_match_pair.begin();
       iter != sc_dist_to_match_pair.end(); iter++) {
    const MatchPair& best_match = iter->second;
    const auto& best_submap_match =
        search_submaps.at(best_match.candidate_submap_id);
    const auto& T_World_QuerySubmap = use_initial_poses
                                          ? query_submap->T_WORLD_SUBMAP_INIT()
                                          : query_submap->T_WORLD_SUBMAP();
    const auto& T_World_MatchSubmap =
        use_initial_poses ? best_submap_match->T_WORLD_SUBMAP_INIT()
                          : best_submap_match->T_WORLD_SUBMAP();

    const auto& scan_candidate = best_match.candidate_scan;
    const auto& scan_query = query_submap_scans.at(best_match.query_scan_id);

    auto scan_pose_candidate_iter = best_submap_match->LidarKeyframesBegin();
    std::advance(scan_pose_candidate_iter, best_match.candidate_scan_id);
    auto T_SubmapCandidate_Lidar =
        scan_pose_candidate_iter->second.T_REFFRAME_LIDAR();

    auto scan_pose_query_iter = query_submap->LidarKeyframesBegin();
    std::advance(scan_pose_query_iter, best_match.query_scan_id);
    auto T_SubmapQuery_Lidar = scan_pose_query_iter->second.T_REFFRAME_LIDAR();

    BEAM_INFO("Aligning aggregate scans between query submap with timestamp {} "
              "and candidate submap with timestamp {}",
              std::to_string(query_submap->Stamp().toSec()),
              std::to_string(best_submap_match->Stamp().toSec()));

    std::string curr_dir;
    if (!output_path.empty()) {
      curr_dir = beam::CombinePaths(
          output_path, std::to_string(query_submap->Stamp().toSec()));
      std::filesystem::create_directory(curr_dir);
    }

    auto maybeT_Candidate_Query = AlignSubmapsFromScanMatches(
        scan_candidate, scan_query, T_SubmapCandidate_Lidar,
        T_SubmapQuery_Lidar, T_World_MatchSubmap, T_World_QuerySubmap,
        curr_dir);

    if (!maybeT_Candidate_Query) {
      BEAM_WARN("Scan matching failed, trying next best candidate");
      continue;
    }

    matched_indices.emplace_back(best_match.candidate_submap_id);
    Ts_Candidate_Query.emplace_back(maybeT_Candidate_Query.value());
    break;
  }
}

std::optional<Eigen::Matrix4d>
    RelocCandidateSearchScanContext::AlignSubmapsFromScanMatches(
        const PointCloudSCPtr& scan_candidate,
        const PointCloudSCPtr& scan_query,
        const Eigen::Matrix4d& T_SubmapCandidate_Lidar,
        const Eigen::Matrix4d& T_SubmapQuery_Lidar,
        const Eigen::Matrix4d& T_World_CandidateSubmap,
        const Eigen::Matrix4d& T_World_QuerySubmap,
        const std::string& output_path) const {
  // get all needed transforms
  Eigen::Matrix4d T_CandidateSubmapEst_QuerySubmap =
      beam::InvertTransform(T_World_CandidateSubmap) * T_World_QuerySubmap;

  // convert to PointCloud
  PointCloud scan_candidate_converted;
  pcl::copyPointCloud(*scan_candidate, scan_candidate_converted);
  PointCloud scan_query_converted;
  pcl::copyPointCloud(*scan_query, scan_query_converted);

  // filter clouds
  PointCloud scan_candidate_filtered =
      beam_filtering::FilterPointCloud<pcl::PointXYZ>(scan_candidate_converted,
                                                      filters_);
  PointCloud scan_query_filtered =
      beam_filtering::FilterPointCloud<pcl::PointXYZ>(scan_query_converted,
                                                      filters_);

  // transform candidate scan into candidate submap frame
  PointCloudPtr candidate_in_candidate_submap_frame =
      std::make_shared<PointCloud>();
  pcl::transformPointCloud(scan_candidate_filtered,
                           *candidate_in_candidate_submap_frame,
                           Eigen::Affine3d(T_SubmapCandidate_Lidar));

  // transform query scan into candidate submap frame
  PointCloudPtr query_in_candidate_submap_frame =
      std::make_shared<PointCloud>();
  pcl::transformPointCloud(
      scan_query_filtered, *query_in_candidate_submap_frame,
      Eigen::Affine3d(T_CandidateSubmapEst_QuerySubmap * T_SubmapQuery_Lidar));

  // align to get transform between them
  matcher_->SetRef(candidate_in_candidate_submap_frame);
  matcher_->SetTarget(query_in_candidate_submap_frame);
  bool match_success = matcher_->Match();
  Eigen::Matrix4d T_Candidate_Query =
      matcher_->ApplyResult(T_CandidateSubmapEst_QuerySubmap);
  if (!output_path.empty()) {
    matcher_->SaveResults(output_path, "candidate_cloud_");
  }

  if (match_success) {
    BEAM_INFO("match successful");
    return T_Candidate_Query;
  } else {
    BEAM_WARN("match unsuccessful");
    return {};
  }
}

std::vector<PointCloudSCPtr>
    RelocCandidateSearchScanContext::AggregateSubmapScans(
        const global_mapping::SubmapPtr& submap) const {
  std::vector<PointCloudSCPtr> clouds;
  const std::map<uint64_t, ScanPose>& lidar_keyframes =
      submap->LidarKeyframes();
  for (int i = 0; i < lidar_keyframes.size(); i++) {
    auto curr_scan_pose_iter = lidar_keyframes.begin();
    std::advance(curr_scan_pose_iter, i);
    PointCloudSCPtr cloud = std::make_shared<PointCloudSC>();
    pcl::copyPointCloud(curr_scan_pose_iter->second.Cloud(), *cloud);
    auto T_Submap_ScanCenter = curr_scan_pose_iter->second.T_REFFRAME_LIDAR();

    std::vector<uint64_t> times = GetTimesToAggregate(lidar_keyframes, i);
    for (const uint64_t timestamp : times) {
      const auto& scan_pose_to_agg = lidar_keyframes.at(timestamp);
      auto T_Submap_ScanToAgg = scan_pose_to_agg.T_REFFRAME_LIDAR();
      auto T_ScanCenter_ScanToAgg =
          beam::InvertTransform(T_Submap_ScanCenter) * T_Submap_ScanToAgg;
      PointCloud cloud_new;
      pcl::transformPointCloud(scan_pose_to_agg.Cloud(), cloud_new,
                               Eigen::Affine3d(T_ScanCenter_ScanToAgg));
      PointCloudSCPtr cloud_new_conv = std::make_shared<PointCloudSC>();
      pcl::copyPointCloud(cloud_new, *cloud_new_conv);
      *cloud += *cloud_new_conv;
    }
    clouds.push_back(cloud);
  }
  return clouds;
}

std::vector<uint64_t> RelocCandidateSearchScanContext::GetTimesToAggregate(
    const std::map<uint64_t, ScanPose>& keyframes, int center_id) const {
  // first, get the IDs we will keep
  std::vector<int> ids;
  int curr_id = center_id - num_scans_to_aggregate_ / 2;
  while (curr_id <= center_id + num_scans_to_aggregate_ / 2 &&
         curr_id < keyframes.size()) {
    if (curr_id >= 0 && curr_id != center_id) { ids.push_back(curr_id); }
    curr_id++;
  }

  // next, lookup their timestamps
  std::vector<uint64_t> times;
  for (const int id : ids) {
    auto iter = keyframes.begin();
    std::advance(iter, id);
    times.push_back(iter->first);
  }
  return times;
}

} // namespace bs_models::reloc
