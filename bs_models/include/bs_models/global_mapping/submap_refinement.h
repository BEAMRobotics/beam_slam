#pragma once

#include <bs_models/global_mapping/utils.h>

namespace bs_models::global_mapping {

class SubmapRefinement {
public:
  struct Params {
    /** Full path to config file for scan registration. If blank, it will use
     * default parameters.*/
    std::string scan_registration_config;

    /** Full path to config file for matcher. If blank, it will use default
     * parameters.*/
    std::string matcher_config;
  };

  SubmapRefinement() = delete;

  SubmapRefinement(const Params& params, const std::string& output_path = "");

  bool Run(std::vector<SubmapPtr> submaps);

  RegistrationResults GetResults() const { return results_; }

private:
  bool RefineSubmap(SubmapPtr& submap);

  Params params_;
  RegistrationResults results_;
  std::string output_path_;
  double prior_cov_multiplyer_{1};
};

} // namespace bs_models::global_mapping