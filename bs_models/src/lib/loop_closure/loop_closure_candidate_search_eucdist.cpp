#include <bs_models/loop_closure/loop_closure_candidate_search_eucdist.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <beam_utils/log.h>
#include <beam_utils/math.h>

#include <bs_common/utils.h>

namespace bs_models {

namespace loop_closure {

LoopClosureCandidateSearchEucDist::LoopClosureCandidateSearchEucDist(
    const std::string& config_path) {
      this->config_path_ = config_path;
      LoadConfig();
    }

LoopClosureCandidateSearchEucDist::LoopClosureCandidateSearchEucDist(
    double distance_threshold_m)
    : distance_threshold_m_(distance_threshold_m) {}

void LoopClosureCandidateSearchEucDist::LoadConfig() {
  std::string read_path = config_path_;

  if (read_path.empty()) {
    return;
  }

  if (read_path == "DEFAULT_PATH") {
    read_path = bs_common::GetBeamSlamConfigPath() +
                "global_map/loop_closure_candidate_search_eucdist.json";
  }

  nlohmann::json J;
  BEAM_INFO("Loading loop closure config: {}", read_path);
  if (!beam::ReadJson(read_path, J)) {
    BEAM_INFO("Using default params.");
    return;
  }

  try {
    distance_threshold_m_ = J["distance_threshold_m"];
  } catch (...) {
    BEAM_ERROR(
        "Missing one or more parameter, using default loop closure "
        "candidate search params");
  }
}

void LoopClosureCandidateSearchEucDist::FindLoopClosureCandidates(
    const std::vector<std::shared_ptr<Submap>>& submaps, int query_index,
    std::vector<int>& matched_indices,
    std::vector<Eigen::Matrix4d, pose_allocator>& estimated_poses) {
  matched_indices.clear();
  estimated_poses.clear();
  const Eigen::Matrix4d& T_WORLD_QUERY =
      submaps.at(query_index)->T_WORLD_SUBMAP();
  for (int i = 0; i < query_index - 1; i++) {
    if (i == query_index) {
      continue;
    }
    const Eigen::Matrix4d& T_WORLD_MATCHCANDIDATE =
        submaps.at(i)->T_WORLD_SUBMAP();
    Eigen::Matrix4d T_MATCHCANDIDATE_QUERY =
        beam::InvertTransform(T_WORLD_MATCHCANDIDATE) * T_WORLD_QUERY;
    double distance = T_MATCHCANDIDATE_QUERY.block(0, 3, 3, 1).norm();
    if (distance < distance_threshold_m_) {
      matched_indices.push_back(i);
      estimated_poses.push_back(T_MATCHCANDIDATE_QUERY);
    }
  }
}

}  // namespace loop_closure

}  // namespace bs_models
