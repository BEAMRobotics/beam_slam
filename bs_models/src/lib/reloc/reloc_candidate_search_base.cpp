#include <bs_models/reloc/reloc_candidate_search_base.h>

#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>
#include <bs_common/utils.h>

#include <bs_models/reloc/reloc_candidate_search_eucdist.h>

namespace bs_models::reloc {

std::shared_ptr<RelocCandidateSearchBase>
    RelocCandidateSearchBase::Create(const std::string& config_path) {
  if (config_path.empty()) {
    BEAM_INFO(
        "No config file provided to RelocCandidateSearchBase, using EUCDIST.");
    return std::make_shared<RelocCandidateSearchEucDist>(config_path);
  }

  BEAM_INFO("Loading config file: {}", config_path);
  nlohmann::json J;
  if (!beam::ReadJson(config_path, J)) {
    BEAM_ERROR("Unable to read config");
    throw std::runtime_error{"Unable to read config"};
  }
  bs_common::ValidateJsonKeysOrThrow(std::vector<std::string>{"type"}, J);

  std::string type = J["type"];
  if (type == "EUCDIST") {
    return std::make_shared<RelocCandidateSearchEucDist>(config_path);
  } else {
    BEAM_ERROR("Invalid type: {}, options: EUCDIST");
    throw std::runtime_error{"invalid type in json"};
  }
}

} // namespace bs_models::reloc
