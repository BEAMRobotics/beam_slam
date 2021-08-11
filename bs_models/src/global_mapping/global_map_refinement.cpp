#include <bs_models/global_mapping/global_map_refinement.h>

#include <bs_models/global_mapping/loop_closure/loop_closure_methods.h>
#include <bs_common/utils.h>

namespace bs_models {

namespace global_mapping {

GlobalMapRefinement::Params::Params() {
  double local_map_cov_diag = 1e-3;
  double loop_cov_diag = 1e-5;

  // clang-format off
  local_mapper_covariance << local_map_cov_diag, 0, 0, 0, 0, 0,
                             0, local_map_cov_diag, 0, 0, 0, 0,
                             0, 0, local_map_cov_diag, 0, 0, 0,
                             0, 0, 0, local_map_cov_diag, 0, 0,
                             0, 0, 0, 0, local_map_cov_diag, 0,
                             0, 0, 0, 0, 0, local_map_cov_diag;

  loop_closure_covariance << loop_cov_diag, 0, 0, 0, 0, 0,
                             0, loop_cov_diag, 0, 0, 0, 0,
                             0, 0, loop_cov_diag, 0, 0, 0,
                             0, 0, 0, loop_cov_diag, 0, 0,
                             0, 0, 0, 0, loop_cov_diag, 0,
                             0, 0, 0, 0, 0, loop_cov_diag;
  // clang-format on
}

void GlobalMapRefinement::Params::LoadJson(const std::string& config_path) {
  std::string read_file = config_path;
  if (read_file.empty()) {
    BEAM_INFO(
        "No config file provided to global map refinement, using default "
        "parameters.");
    return;
  }

  if (read_file == "DEFAULT_PATH") {
    read_file = bs_common::GetBeamSlamConfigPath() +
                "global_map/global_map_refinement.json";
  }

  if (!boost::filesystem::exists(read_file)) {
    BEAM_ERROR(
        "Cannot find global map refinement config at: {}, using default "
        "parameters.",
        read_file);
    return;
  }

  BEAM_INFO("Loading global map config file: {}", read_file);

  nlohmann::json J;
  std::ifstream file(read_file);
  file >> J;
  loop_closure_candidate_search_type = J["loop_closure_candidate_search_type"];
  loop_closure_refinement_type = J["loop_closure_refinement_type"];
  loop_closure_candidate_search_config =
      J["loop_closure_candidate_search_config"];
  loop_closure_refinement_config = J["loop_closure_refinement_config"];

  std::vector<double> vec = J["local_mapper_covariance_diag"];
  if (vec.size() != 6) {
    BEAM_ERROR(
        "Invalid local mapper covariance diagonal (6 values required). Using "
        "default.");
  } else {
    Eigen::VectorXd vec_eig(6);
    vec_eig << vec[0], vec[1], vec[2], vec[3], vec[4], vec[5];
    local_mapper_covariance = vec_eig.asDiagonal();
  }

  std::vector<double> vec2 = J["loop_closure_covariance_diag"];
  if (vec2.size() != 6) {
    BEAM_ERROR(
        "Invalid loop closure covariance diagonal (6 values required). Using "
        "default.");
  } else {
    Eigen::VectorXd vec_eig = Eigen::VectorXd(6);
    vec_eig << vec2[0], vec2[1], vec2[2], vec2[3], vec2[4], vec2[5];
    loop_closure_covariance = vec_eig.asDiagonal();
  }
}

void GlobalMapRefinement::Params::SaveJson(const std::string& filename) {
  nlohmann::json J = {
      {"loop_closure_candidate_search_type",
       loop_closure_candidate_search_type},
      {"loop_closure_refinement_type", loop_closure_refinement_type},
      {"loop_closure_candidate_search_config",
       loop_closure_candidate_search_config},
      {"local_mapper_covariance_diag",
       {local_mapper_covariance(0, 0), local_mapper_covariance(1, 1),
        local_mapper_covariance(2, 2), local_mapper_covariance(3, 3),
        local_mapper_covariance(4, 4), local_mapper_covariance(5, 5)}},
      {"loop_closure_covariance_diag",
       {loop_closure_covariance(0, 0), loop_closure_covariance(1, 1),
        loop_closure_covariance(2, 2), loop_closure_covariance(3, 3),
        loop_closure_covariance(4, 4), loop_closure_covariance(5, 5)}}};

  std::ofstream file(filename);
  file << std::setw(4) << J << std::endl;
}

GlobalMapRefinement::GlobalMapRefinement(std::vector<Submap>& submaps)
    : submaps_(submaps) {
  Setup();
}

GlobalMapRefinement::GlobalMapRefinement(std::vector<Submap>& submaps,
                                         const Params& params)
    : submaps_(submaps), params_(params) {
  Setup();
}

GlobalMapRefinement::GlobalMapRefinement(std::vector<Submap>& submaps,
                                         const std::string& config_path)
    : submaps_(submaps) {
  params_.LoadJson(config_path);
  Setup();
}

void GlobalMapRefinement::Setup() {
  // initiate loop closure candidate search
  if (params_.loop_closure_candidate_search_type == "EUCDIST") {
    loop_closure_candidate_search_ =
        std::make_unique<LoopClosureCandidateSearchEucDist>(
            params_.loop_closure_candidate_search_config);
  } else {
    BEAM_ERROR(
        "Invalid loop closure candidate search type. Using default: EUCDIST. "
        "Input: {}",
        params_.loop_closure_candidate_search_type);
    loop_closure_candidate_search_ =
        std::make_unique<LoopClosureCandidateSearchEucDist>(
            params_.loop_closure_candidate_search_config);
  }

  // initiate loop closure refinement
  if (params_.loop_closure_refinement_type == "ICP") {
    loop_closure_refinement_ = std::make_unique<LoopClosureRefinementIcp>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  } else if (params_.loop_closure_refinement_type == "GICP") {
    loop_closure_refinement_ = std::make_unique<LoopClosureRefinementGicp>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  } else if (params_.loop_closure_refinement_type == "NDT") {
    loop_closure_refinement_ = std::make_unique<LoopClosureRefinementNdt>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  } else if (params_.loop_closure_refinement_type == "LOAM") {
    loop_closure_refinement_ = std::make_unique<LoopClosureRefinementLoam>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  } else {
    BEAM_ERROR("Invalid loop closure refinement type. Using default: ICP");
    loop_closure_refinement_ = std::make_unique<LoopClosureRefinementIcp>(
        params_.loop_closure_covariance,
        params_.loop_closure_refinement_config);
  }
}

bool GlobalMapRefinement::RunSubmapRefinement() {
  for(uint16_t i = 0; i < submaps_.size(); i++){
    BEAM_INFO("Refining submap No. {}", static_cast<int>(i));
    if(!RefineSubmap(submaps_[i])){
      BEAM_ERROR("Submap refinement failed, exiting.");
      return false;
    }
  }
  return true;
}

bool GlobalMapRefinement::RefineSubmap(Submap& submap){
  // TODO
}

bool GlobalMapRefinement::RunPoseGraphOptimization() {
  // TODO
}

}  // namespace global_mapping

}  // namespace bs_models