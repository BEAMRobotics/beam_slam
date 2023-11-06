#include <bs_models/reloc/reloc_refinement_base.h>

#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>

#include <bs_common/utils.h>
#include <bs_models/reloc/reloc_refinement_loam_registration.h>
#include <bs_models/reloc/reloc_refinement_scan_registration.h>

namespace bs_models::reloc {

std::shared_ptr<RelocRefinementBase>
    RelocRefinementBase::Create(const std::string& config_path) {
  if (config_path.empty()) {
    BEAM_INFO("No config file provided to RelocRefinementBase, using ICP.");
    return std::make_shared<RelocRefinementIcp>(config_path);
  }

  BEAM_INFO("Loading config file: {}", config_path);
  nlohmann::json J;
  if (!beam::ReadJson(config_path, J)) {
    BEAM_ERROR("Unable to read config");
    throw std::runtime_error{"Unable to read config"};
  }
  beam::ValidateJsonKeysOrThrow({"type"}, J);

  std::string type = J["type"];
  if (type == "ICP") {
    return std::make_shared<RelocRefinementIcp>(config_path);
  } else if (type == "GICP") {
    BEAM_ERROR("GICP NOT IMPLEMENTED - Bug with includes");
    throw std::runtime_error{"invalid type in json"};
    // TODO: Fix the compile error when we uncomment this
    // return std::make_shared<RelocRefinementGicp>(config_path);
  } else if (type == "NDT") {
    return std::make_shared<RelocRefinementNdt>(config_path);
  } else if (type == "LOAM") {
    return std::make_shared<RelocRefinementLoam>(config_path);
  } else {
    BEAM_ERROR("Invalid type: {}, options: ICP, GICP, NDT, LOAM", type);
    throw std::runtime_error{"invalid type in json"};
  }
}

} // namespace bs_models::reloc
