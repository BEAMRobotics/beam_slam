#pragma once

#include <map>

#include <ros/time.h>
#include <fuse_core/transaction.h>

#include <beam_utils/pointclouds.h>
#include <beam_matching/Matchers.h>
#include <bs_models/global_mapping/loop_closure/loop_closure_refinement_base.h>
#include <bs_constraints/frame_to_frame/pose_3d_stamped_transaction.h>
#include <bs_models/global_mapping/submap.h>

using namespace beam_matching;

namespace bs_models {

namespace global_mapping {

/**
 * @brief Templated class for loop closure refinement with lidar scan matching
 */
template <typename MatcherType, typename ParamsType>
class LoopClosureRefinementScanRegistration : public LoopClosureRefinementBase {
 public:
  // Inherit base class constructors
  using LoopClosureRefinementBase::LoopClosureRefinementBase;

  /**
   * @brief another constructor that takes in parameters
   * @param matcher_config full path to matcher config json
   */
  LoopClosureRefinementScanRegistration(
      const Eigen::Matrix<double, 6, 6>& loop_closure_covariance,
      const std::string& matcher_config = "")
      : loop_closure_covariance_(loop_closure_covariance),
        matcher_config_(matcher_config) {
    covariance_set_ = true;
  }

  /**
   * @brief Generate a fuse transaction between two candidate loop closure
   * submaps
   * @param matched_submap submap that a new query submap matches to
   * @param query_submap new submap that we are adding constraints with previous
   * submaps
   * @param T_MATCH_QUERY_EST estimated transform between the two submaps. This
   * is determined with a class derived from LoopClosureCandidateSearchBase
   */
  fuse_core::Transaction::SharedPtr GenerateTransaction(
      const Submap& matched_submap, const Submap& query_submap,
      const Eigen::Matrix4d& T_MATCH_QUERY_EST) override {
    LoadConfig();
    Setup();

    Eigen::Matrix4d T_MATCH_QUERY_OPT;
    if (!GetRefinedT_MATCH_QUERY(matched_submap, query_submap,
                                 T_MATCH_QUERY_EST, T_MATCH_QUERY_OPT)) {
      return nullptr;
    }

    std::string source = "LOOPCLOSURE";
    bs_constraints::frame_to_frame::Pose3DStampedTransaction transaction(
        query_submap.Stamp());
    transaction.AddPoseConstraint(
        matched_submap.T_WORLD_SUBMAP(), query_submap.T_WORLD_SUBMAP(),
        matched_submap.Stamp(), query_submap.Stamp(), T_MATCH_QUERY_OPT,
        loop_closure_covariance_, source);

    return transaction.GetTransaction();
  }

 private:
  /**
   * @brief Method for loading a config json file.
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

  /**
   * @brief initiate variables
   */
  void Setup() {
    matcher_ = std::make_unique<MatcherType>(ParamsType(matcher_config_));
    if (!covariance_set_) {
      Eigen::VectorXd vec(6);
      vec << 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5;
      loop_closure_covariance_ = vec.asDiagonal();
      covariance_set_ = true;
    }
  }

  /**
   * @brief Calculate a refined pose between submaps using scan registration
   * @param matched_submap submap that a new query submap matches to
   * @param query_submap new submap that we are adding constraints with previous
   * submaps
   * @param T_MATCH_QUERY_EST estimated transform between the two submaps. This
   * is determined with a class derived from LoopClosureCandidateSearchBase
   * @param T_MATCH_QUERY_OPT reference to the resulting refined transform from
   * query submap to matched submap
   */
  bool GetRefinedT_MATCH_QUERY(const Submap& matched_submap,
                               const Submap& query_submap,
                               const Eigen::Matrix4d& T_MATCH_QUERY_EST,
                               Eigen::Matrix4d& T_MATCH_QUERY_OPT) {
    PointCloudPtr cloud_match_world = std::make_shared<PointCloud>();
    *cloud_match_world = matched_submap.GetLidarPointsInWorldFrame();
    PointCloudPtr cloud_query_world = std::make_shared<PointCloud>();
    *cloud_query_world = query_submap.GetLidarPointsInWorldFrame();

    matcher_->SetRef(cloud_match_world);
    matcher_->SetTarget(cloud_query_world);
    if (!matcher_->Match()) {
      BEAM_WARN("Failed scan matching. Not adding loop closure constraint.");
      return false;
    }
    T_MATCH_QUERY_OPT = matcher_->GetResult().inverse().matrix();
  }

  std::unique_ptr<Matcher<PointCloudPtr>> matcher_;
  std::string matcher_config_{""};
  bool covariance_set_{false};
  Eigen::Matrix<double, 6, 6> loop_closure_covariance_;
};

using LoopClosureRefinementIcp =
    LoopClosureRefinementScanRegistration<IcpMatcher, IcpMatcher::Params>;
using LoopClosureRefinementGicp =
    LoopClosureRefinementScanRegistration<GicpMatcher, GicpMatcher::Params>;
using LoopClosureRefinementNdt =
    LoopClosureRefinementScanRegistration<NdtMatcher, NdtMatcher::Params>;

}  // namespace global_mapping

}  // namespace bs_models
