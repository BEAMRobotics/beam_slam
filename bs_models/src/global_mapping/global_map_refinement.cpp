#include <bs_models/global_mapping/global_map_refinement.h>

namespace bs_models {

namespace global_mapping {

void GlobalMapRefinement::Params::LoadJson(const std::string& config_path) {
  // TODO
}

void GlobalMapRefinement::Params::SaveJson(const std::string& filename) {
  // TODO
}

GlobalMapRefinement::GlobalMapRefinement(std::vector<Submap>& submaps) : submaps_(submaps) {
  // TODO
}

GlobalMapRefinement::GlobalMapRefinement(std::vector<Submap>& submaps,
                                         const Params& params) : submaps_(submaps) {
  // TODO
}

GlobalMapRefinement::GlobalMapRefinement(std::vector<Submap>& submaps,
                                         const std::string& config_path) : submaps_(submaps) {
  // TODO
}

void GlobalMapRefinement::Setup() {
  // TODO
}

bool GlobalMapRefinement::RunSubmapRefinement() {
  // TODO
}

bool GlobalMapRefinement::RunPoseGraphOptimization() {
  // TODO
}

}  // namespace global_mapping

}  // namespace bs_models