#include <bs_models/reloc/reloc_candidate_search_scan_context.h>

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

  bs_common::ValidateJsonKeysOrThrow(
      std::vector<std::string>{"type", "submap_distance_threshold_m",
                               "matcher_config"},
      J);
  std::string type = J["type"];
  if (type != "SCANCONTEXT") {
    BEAM_ERROR(
        "Invalid config file provided to RelocCandidateSearchScanContext: {}",
        type);
    throw std::runtime_error{"invalid config file"};
  }

  submap_distance_threshold_m_ = J["submap_distance_threshold_m"];

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
    matcher_loam_ = std::make_unique<LoamMatcher>(LoamParams(matcher_config));
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
    size_t ignore_last_n_submaps, bool use_initial_poses) {
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

  // build map from scan context scores to match pairs for all candidate submaps
  std::map<float, MatchPair> score_to_match_pair;
  for (const auto& [distance, submap_id] : initial_candidates_sorted) {
    // iterate through scans in candidate submap and build a scan context
    // database
    SCManager sc_manager;
    std::vector<uint64_t> match_scan_timestamps;
    for (auto scan_iter = search_submaps.at(submap_id)->LidarKeyframesBegin();
         scan_iter != search_submaps.at(submap_id)->LidarKeyframesEnd();
         scan_iter++) {
      const auto& scan_pose = scan_iter->second;
      pcl::PointCloud<pcl::PointXYZI> input_cloud;
      pcl::copyPointCloud(scan_pose.Cloud(), input_cloud);
      sc_manager.makeAndSaveScancontextAndKeys(input_cloud);
      match_scan_timestamps.push_back(scan_pose.Stamp().toNSec());
    }

    // iterate through scans in query submap, and find best match in candidate
    // submap
    for (auto scan_iter = query_submap->LidarKeyframesBegin();
         scan_iter != query_submap->LidarKeyframesEnd(); scan_iter++) {
      const auto& scan_pose = scan_iter->second;
      pcl::PointCloud<pcl::PointXYZI> input_cloud;
      pcl::copyPointCloud(scan_pose.Cloud(), input_cloud);
      std::pair<int, float> scan_id_and_dist =
          sc_manager.detectLoopClosureID(input_cloud);
      MatchPair match_pair;
      match_pair.matched_submap_scan_time =
          match_scan_timestamps.at(scan_id_and_dist.first);
      match_pair.query_submap_scan_time = scan_pose.Stamp().toNSec();
      match_pair.matched_submap_id = submap_id;
      score_to_match_pair.emplace(scan_id_and_dist.second, match_pair);
    }
  }

  // align the math pair with the highest score
  for (auto iter = score_to_match_pair.rbegin();
       iter != score_to_match_pair.rend(); iter++) {
    const auto& best_match = *iter;
    const auto& best_submap_match =
        search_submaps.at(best_match.second.matched_submap_id);
    const auto& scan_pose_match = best_submap_match->LidarKeyframes().at(
        best_match.second.matched_submap_scan_time);

    const auto& scan_pose_query = query_submap->LidarKeyframes().at(
        best_match.second.query_submap_scan_time);
    const auto& T_World_QuerySubmap = use_initial_poses
                                          ? query_submap->T_WORLD_SUBMAP_INIT()
                                          : query_submap->T_WORLD_SUBMAP();
    const auto& T_World_MatchSubmap =
        use_initial_poses ? best_submap_match->T_WORLD_SUBMAP_INIT()
                          : best_submap_match->T_WORLD_SUBMAP();

    auto maybeT_Candidate_Query =
        AlignSubmapsFromScanMatches(scan_pose_match, scan_pose_query,
                                    T_World_MatchSubmap, T_World_QuerySubmap);

    if (!maybeT_Candidate_Query) {
      BEAM_WARN("Scan matching failed, trying next best candidate");
      continue;
    }

    matched_indices.clear();
    matched_indices.emplace_back(best_match.second.matched_submap_id);
    Ts_Candidate_Query.clear();
    Ts_Candidate_Query.emplace_back(maybeT_Candidate_Query.value());
    break;
  }
}

std::optional<Eigen::Matrix4d>
    RelocCandidateSearchScanContext::AlignSubmapsFromScanMatches(
        const ScanPose& scan_pose_candidate, const ScanPose& scan_pose_query,
        const Eigen::Matrix4d& T_World_CandidateSubmap,
        const Eigen::Matrix4d& T_World_QuerySubmap) const {
  // get all needed transforms
  Eigen::Matrix4d T_CandidateSub_Lidar = scan_pose_candidate.T_REFFRAME_LIDAR();
  Eigen::Matrix4d T_QuerySub_Lidar = scan_pose_query.T_REFFRAME_LIDAR();
  Eigen::Matrix4d T_CandidateSubmapEst_QuerySubmap =
      beam::InvertTransform(T_World_CandidateSubmap) * T_World_QuerySubmap;

  Eigen::Matrix4d T_CandidateSub_QuerySub;
  bool match_success;
  if (matcher_loam_) {
    // transform candidate scan into candidate submap frame
    LoamPointCloudPtr candidate_in_candidate_submap_frame =
        std::make_shared<LoamPointCloud>(scan_pose_candidate.LoamCloud());
    candidate_in_candidate_submap_frame->TransformPointCloud(
        T_CandidateSub_Lidar);

    // transform query scan into candidate submap frame
    LoamPointCloudPtr query_in_candidate_submap_frame =
        std::make_shared<LoamPointCloud>(scan_pose_query.LoamCloud());
    query_in_candidate_submap_frame->TransformPointCloud(
        T_CandidateSubmapEst_QuerySubmap * T_QuerySub_Lidar);

    // align to get transform between them
    matcher_loam_->SetRef(candidate_in_candidate_submap_frame);
    matcher_loam_->SetTarget(query_in_candidate_submap_frame);
    match_success = matcher_loam_->Match();
    T_CandidateSub_QuerySub = matcher_loam_->GetResult().inverse().matrix();
  } else {
    // transform candidate scan into candidate submap frame
    PointCloud candidate_in_candidate_submap_frame;
    pcl::transformPointCloud(scan_pose_candidate.Cloud(),
                             candidate_in_candidate_submap_frame,
                             Eigen::Affine3d(T_CandidateSub_Lidar));

    // transform query scan into candidate submap frame
    PointCloud query_in_candidate_submap_frame;
    pcl::transformPointCloud(
        scan_pose_query.Cloud(), query_in_candidate_submap_frame,
        Eigen::Affine3d(T_CandidateSubmapEst_QuerySubmap * T_QuerySub_Lidar));

    // align to get transform between them
    matcher_->SetRef(
        std::make_shared<PointCloud>(candidate_in_candidate_submap_frame));
    matcher_->SetTarget(
        std::make_shared<PointCloud>(query_in_candidate_submap_frame));
    match_success = matcher_->Match();
    T_CandidateSub_QuerySub = matcher_->GetResult().inverse().matrix();
  }

  if (match_success) {
    return T_CandidateSub_QuerySub;
  } else {
    return {};
  }
}

} // namespace bs_models::reloc
