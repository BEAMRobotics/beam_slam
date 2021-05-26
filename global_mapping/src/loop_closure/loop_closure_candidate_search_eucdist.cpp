#include <global_mapping/loop_closure/loop_closure_candidate_search_eucdist.h>

#include <boost/filesystem.hpp>

#include <beam_utils/log.h>

namespace global_mapping {

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

  // TODO load params
}

void LoopClosureCandidateSearchEucDist::FindLoopClosureCandidates(
    const std::vector<Submap>& submaps, int current_index,
    std::vector<int>& matched_indices,
    std::vector<Eigen::Matrix4d, pose_allocator>& estimated_poses) {
  // TODO
}

}  // namespace global_mapping