#pragma once

#include <bs_models/global_mapping/utils.h>

namespace bs_models::global_mapping {

class SubmapPoseGraphOptimization {
public:
  struct Params {
    /** Full path to config file for loop closure candidate search. If blank, it
     * will use default parameters.*/
    std::string candidate_search_config;

    /** Full path to config file for loop closure refinement. If blank, it will
     * use default parameters.*/
    std::string refinement_config;

    /** Weights to assign to loop closure measurements */
    Eigen::Matrix<double, 6, 6> loop_closure_covariance;

    /** Weights to assign to local mapper measurements */
    Eigen::Matrix<double, 6, 6> local_mapper_covariance;
  };

  SubmapPoseGraphOptimization() = delete;

  SubmapPoseGraphOptimization(const Params& params,
                              const std::string& output_path = "");

  ~SubmapPoseGraphOptimization() = default;

  bool Run(std::vector<SubmapPtr> submaps);

private:
  Params params_;
  std::string output_path_;

  // params only tunable here
  int pgo_skip_first_n_submaps_{2};
  double pose_prior_noise_fixed_{1e-9};
};

} // namespace bs_models::global_mapping