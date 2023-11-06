#include <bs_models/reloc/reloc_candidate_search_eucdist.h>

#include <nlohmann/json.hpp>

#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <bs_common/utils.h>

namespace bs_models::reloc {

RelocCandidateSearchEucDist::RelocCandidateSearchEucDist(
    const std::string& config)
    : config_path_(config) {
  LoadConfig();
}

RelocCandidateSearchEucDist::RelocCandidateSearchEucDist(
    double distance_threshold_m)
    : distance_threshold_m_(distance_threshold_m) {}

void RelocCandidateSearchEucDist::LoadConfig() {
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

  beam::ValidateJsonKeysOrThrow({"type"}, J);
  std::string type = J["type"];
  if (type != "EUCDIST") {
    BEAM_ERROR(
        "Invalid config file provided to RelocCandidateSearchScanContext: {}",
        type);
    throw std::runtime_error{"invalid config file"};
  }

  beam::ValidateJsonKeysOrThrow({"distance_threshold_m"}, J);
  distance_threshold_m_ = J["distance_threshold_m"];
}

void RelocCandidateSearchEucDist::FindRelocCandidates(
    const std::vector<global_mapping::SubmapPtr>& search_submaps,
    const global_mapping::SubmapPtr& query_submap,
    std::vector<int>& matched_indices,
    std::vector<Eigen::Matrix4d, beam::AlignMat4d>& Ts_Candidate_Query,
    size_t ignore_last_n_submaps, bool use_initial_poses,
    const std::string& output_path) {
  if (search_submaps.size() <= ignore_last_n_submaps) { return; }

  const Eigen::Matrix4d& T_WORLD_QUERY = query_submap->T_WORLD_SUBMAP();

  // create a sorted map to store distances
  std::map<double, std::pair<int, Eigen::Matrix4d>> candidates_sorted;
  for (int i = 0; i < search_submaps.size() - ignore_last_n_submaps; i++) {
    Eigen::Matrix4d T_WORLD_SUBMAPCANDIDATE =
        use_initial_poses ? search_submaps.at(i)->T_WORLD_SUBMAP_INIT()
                          : search_submaps.at(i)->T_WORLD_SUBMAP();
    Eigen::Matrix4d T_SUBMAPCANDIDATE_QUERY =
        beam::InvertTransform(T_WORLD_SUBMAPCANDIDATE) * T_WORLD_QUERY;

    double distance = T_SUBMAPCANDIDATE_QUERY.block(0, 3, 3, 1).norm();

    if (distance < distance_threshold_m_) {
      candidates_sorted.emplace(distance, std::pair<int, Eigen::Matrix4d>(
                                              i, T_SUBMAPCANDIDATE_QUERY));
    }
  }

  // iterate through sorted map and convert to vectors
  matched_indices.clear();
  Ts_Candidate_Query.clear();
  for (auto iter = candidates_sorted.begin(); iter != candidates_sorted.end();
       iter++) {
    matched_indices.push_back(iter->second.first);
    Ts_Candidate_Query.push_back(iter->second.second);
  }

  // todo: output results
}

} // namespace bs_models::reloc
