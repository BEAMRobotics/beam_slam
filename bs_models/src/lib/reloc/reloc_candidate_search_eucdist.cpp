#include <bs_models/reloc/reloc_candidate_search_eucdist.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <bs_common/utils.h>

namespace bs_models { namespace reloc {

RelocCandidateSearchEucDist::RelocCandidateSearchEucDist(
    const std::string& config_path) {
  this->config_path_ = config_path;
  LoadConfig();
}

RelocCandidateSearchEucDist::RelocCandidateSearchEucDist(
    double distance_threshold_m)
    : distance_threshold_m_(distance_threshold_m) {}

void RelocCandidateSearchEucDist::LoadConfig() {
  std::string read_path = config_path_;

  if (read_path.empty()) { return; }

  if (read_path == "DEFAULT_PATH") {
    read_path = bs_common::GetBeamSlamConfigPath() +
                "global_map/reloc_candidate_search_eucdist.json";
  }

  nlohmann::json J;
  BEAM_INFO("Loading reloc config: {}", read_path);
  if (!beam::ReadJson(read_path, J)) {
    BEAM_INFO("Using default params.");
    return;
  }

  try {
    distance_threshold_m_ = J["distance_threshold_m"];
  } catch (...) {
    BEAM_ERROR("Missing one or more parameter, using default reloc "
               "candidate search params");
  }
}

void RelocCandidateSearchEucDist::FindRelocCandidates(
    const std::vector<SubmapPtr>& submaps, const Eigen::Matrix4d& T_WORLD_QUERY,
    const std::vector<cv::Mat>& query_images, std::vector<int>& matched_indices,
    std::vector<Eigen::Matrix4d, beam::AlignMat4d>& estimated_poses,
    size_t ignore_last_n_submaps, bool use_initial_poses) {
  if (submaps.size() <= ignore_last_n_submaps) { return; }

  // create a sorted map to store distances
  std::map<double, std::pair<int, Eigen::Matrix4d>> candidates_sorted;
  for (int i = 0; i < submaps.size() - ignore_last_n_submaps; i++) {
    Eigen::Matrix4d T_WORLD_SUBMAPCANDIDATE;
    if (use_initial_poses) {
      T_WORLD_SUBMAPCANDIDATE = submaps.at(i)->T_WORLD_SUBMAP();
    } else {
      T_WORLD_SUBMAPCANDIDATE = submaps.at(i)->T_WORLD_SUBMAP_INIT();
    }
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
  estimated_poses.clear();
  for (auto iter = candidates_sorted.begin(); iter != candidates_sorted.end();
       iter++) {
    matched_indices.push_back(iter->second.first);
    estimated_poses.push_back(iter->second.second);
  }
}

}} // namespace bs_models::reloc
