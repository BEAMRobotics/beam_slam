#include <bs_models/reloc/reloc_candidate_search_scan_context.h>

#include <nlohmann/json.hpp>

#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <bs_common/utils.h>

namespace bs_models::reloc {

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

  bs_common::ValidateJsonKeysOrThrow(std::vector<std::string>{"type"}, J);
  std::string type = J["type"];
  if (type != "SCANCONTEXT") {
    BEAM_ERROR(
        "Invalid config file provided to RelocCandidateSearchScanContext: {}",
        type);
    throw std::runtime_error{"invalid config file"};
  }
}

void RelocCandidateSearchScanContext::FindRelocCandidates(
    const std::vector<global_mapping::SubmapPtr>& search_submaps,
    const global_mapping::SubmapPtr& query_submap,
    std::vector<int>& matched_indices,
    std::vector<Eigen::Matrix4d, beam::AlignMat4d>& Ts_Candidate_Query,
    size_t ignore_last_n_submaps, bool use_initial_poses) {
  if (search_submaps.size() <= ignore_last_n_submaps) { return; }

  // TODO
}

} // namespace bs_models::reloc
