#include <bs_models/global_mapping/loop_closure/loop_closure_candidate_search_eucdist.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <beam_utils/log.h>
#include <beam_utils/math.h>

namespace bs_models {

namespace global_mapping {

LoopClosureCandidateSearchEucDist::LoopClosureCandidateSearchEucDist(
    double distance_threshold_m)
    : distance_threshold_m_(distance_threshold_m) {}

void LoopClosureCandidateSearchEucDist::LoadConfig() {
  if (config_path_.empty()) {
    return;
  }

  if (!boost::filesystem::exists(config_path_)) {
    BEAM_ERROR(
        "Invalid path to loop closure candidate search config. Using default "
        "parameters. Input: {}",
        config_path_);
    return;
  }

  nlohmann::json J;
  std::ifstream file(config_path_);
  file >> J;
  distance_threshold_m_ = J["distance_threshold_m"];
}

void LoopClosureCandidateSearchEucDist::FindLoopClosureCandidates(
    const std::vector<Submap>& submaps, int query_index,
    std::vector<int>& matched_indices,
    std::vector<Eigen::Matrix4d, pose_allocator>& estimated_poses) {
  LoadConfig();
  matched_indices.clear();
  estimated_poses.clear();
  const Eigen::Matrix4d& T_WORLD_QUERY = submaps[query_index].T_WORLD_SUBMAP();
  for (int i = 0; i < submaps.size(); i++) {
    if (i == query_index) {
      continue;
    }
    const Eigen::Matrix4d& T_WORLD_MATCHCANDIDATE = submaps[i].T_WORLD_SUBMAP();
    Eigen::Matrix4d T_MATCHCANDIDATE_QUERY =
        beam::InvertTransform(T_WORLD_MATCHCANDIDATE) * T_WORLD_QUERY;
    double distance = T_MATCHCANDIDATE_QUERY.block(0, 3, 3, 1).norm();
    if (distance < distance_threshold_m_) {
      matched_indices.push_back(i);
      estimated_poses.push_back(T_MATCHCANDIDATE_QUERY);
    }
  }
}

}  // namespace global_mapping

}  // namespace bs_models
