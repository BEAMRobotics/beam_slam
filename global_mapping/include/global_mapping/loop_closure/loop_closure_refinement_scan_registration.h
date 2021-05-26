#pragma once

#include <map>

#include <ros/time.h>
#include <fuse_core/transaction.h>
#include <nlohmann/json.hpp>

#include <beam_utils/pointclouds.h>
#include <beam_matching/Matchers.h>
#include <global_mapping/loop_closure/loop_closure_refinement_base.h>

namespace global_mapping {

/**
 * @brief Templated class for loop closure refinement with lidar scan matching
 */
template <typename PointCloudType, typename MatcherType>
class LoopClosureRefinementScanRegistration : public LoopClosureRefinementBase {
 public:
  // Inherit base class constructors
  using LoopClosureRefinementBase::LoopClosureRefinementBase;

  /**
   * @brief Generate a fuse transaction between two candidate loop closure
   * submaps
   * @param matched_submap
   * @param query_submap
   */
  fuse_core::Transaction::SharedPtr GenerateTransaction(
      const Submap& matched_submap, const Submap& query_submap) override {
    // TODO
    fuse_core::Transaction::SharedPtr transaction =
        fuse_core::Transaction::make_shared();
    return transaction;
  }

 private:
  /**
   * @brief pure virtual method for loading a config json file.
   */
  void LoadConfig() override {
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

    nlohmann::json J;
    std::ifstream file(config_path_);
    file >> J;
    matcher_config_ = J["matcher_config_path"];
  }

  void Setup() { matcher_ = std::make_unique<MatcherType>(matcher_config_); }

  std::unique_ptr<beam_matching::Matcher<PointCloudType>> matcher_;
  std::string matcher_config_{""};
};

using LoopClosureRefinementIcp =
    LoopClosureRefinementScanRegistration<PointCloudPtr,
                                          beam_matching::IcpMatcher>;
using LoopClosureRefinementGicp =
    LoopClosureRefinementScanRegistration<PointCloudPtr,
                                          beam_matching::GicpMatcher>;
using LoopClosureRefinementNdt =
    LoopClosureRefinementScanRegistration<PointCloudPtr,
                                          beam_matching::NdtMatcher>;
using LoopClosureRefinementLoam =
    LoopClosureRefinementScanRegistration<beam_matching::LoamPointCloudPtr,
                                          beam_matching::NdtMatcher>;

}  // namespace global_mapping