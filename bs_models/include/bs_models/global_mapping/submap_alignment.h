#pragma once

#include <bs_models/global_mapping/utils.h>

namespace bs_models::global_mapping {

class SubmapAlignment {
public:
  struct Params {
    /** Full path to config file for matcher. If blank, it will use default
     * parameters.*/
    std::string matcher_config;
  };

  SubmapAlignment() = delete;

  SubmapAlignment(const Params& params, const std::string& output_path = "");

  ~SubmapAlignment() = default;

  bool Run(std::vector<SubmapPtr> submaps);

  RegistrationResults GetResults() const { return results_; }

private:
  /**
   * @brief Aligns the tgt submap to the reference submap. This is designed to
   * be called one by one starting with the second submap. Since drift builds up
   * over time, we want to always use the initial relative poses between the two
   * submaps to initialize the registration.
   * @param submap_ref const reference to submap to be aligned to
   * @param submap_tgt reference to submap to be aligned
   * @return true if successful
   */
  bool AlignSubmaps(const SubmapPtr& submap_ref, SubmapPtr& submap_tgt);

  Params params_;
  RegistrationResults results_;
  std::string output_path_;
};

} // namespace bs_models::global_mapping