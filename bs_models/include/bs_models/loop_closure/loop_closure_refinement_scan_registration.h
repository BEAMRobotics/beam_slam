#pragma once

#include <map>

#include <ros/time.h>
#include <fuse_core/transaction.h>

#include <beam_utils/pointclouds.h>
#include <beam_matching/Matchers.h>
#include <beam_filtering/Utils.h>

#include <bs_models/loop_closure/loop_closure_refinement_base.h>
#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>

namespace bs_models {

namespace loop_closure {

/**
 * @brief Templated class for loop closure refinement with lidar scan matching
 */
template <typename MatcherType, typename ParamsType>
class LoopClosureRefinementScanRegistration : public LoopClosureRefinementBase {
 public:
  /**
   * @brief constructor requiring only a path to a config file
   * @param config full path to config json
   * @param loop_closure_covariance
   */
  LoopClosureRefinementScanRegistration(
      const Eigen::Matrix<double, 6, 6>& loop_closure_covariance,
      const std::string& config = "")
      : LoopClosureRefinementBase(config),
        loop_closure_covariance_(loop_closure_covariance) {
    LoadConfig();
    Setup();
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
      const std::shared_ptr<global_mapping::Submap>& matched_submap,
      const std::shared_ptr<global_mapping::Submap>& query_submap,
      const Eigen::Matrix4d& T_MATCH_QUERY_EST) override {
    // get refined transform
    Eigen::Matrix4d T_MATCH_QUERY_OPT;
    if (!GetRefinedT_MATCH_QUERY(matched_submap, query_submap,
                                 T_MATCH_QUERY_EST, T_MATCH_QUERY_OPT)) {
      return nullptr;
    }

    // create transaction
    bs_constraints::relative_pose::Pose3DStampedTransaction transaction(
        query_submap->Stamp());
    transaction.AddPoseConstraint(
        matched_submap->T_WORLD_SUBMAP(), query_submap->T_WORLD_SUBMAP(),
        matched_submap->Stamp(), query_submap->Stamp(), T_MATCH_QUERY_OPT,
        loop_closure_covariance_, source_);

    return transaction.GetTransaction();
  }

 private:
  /**
   * @brief Method for loading a config json file.
   */
  void LoadConfig() {
    if (config_path_.empty()) {
      return;
    }

    nlohmann::json J;
    if (!beam::ReadJson(config_path_, J)) {
      BEAM_INFO(
          "Using default loop closure refinement scan registration params.");
      return;
    }

    try {
      matcher_config_ = J["matcher_config_path"];
    } catch (...) {
      BEAM_ERROR(
          "Missing ont or more parameter, using default loop closure "
          "refinement params");
    }

    nlohmann::json J_filters;
    try {
      J_filters = J["filters"];
    } catch (...) {
      ROS_ERROR(
          "Missing 'filters' param in loop closure config file. Not using "
          "filters.");
      return;
    }
    filter_params_ = beam_filtering::LoadFilterParamsVector(J_filters);
    BEAM_INFO("Loaded %d input filters", filter_params_.size());
  }

  /**
   * @param takes all params loaded from LoadConfig, and initializes all member
   * classes/variables
   */
  void Setup() {
    // load matcher
    matcher_ = std::make_unique<MatcherType>(ParamsType(matcher_config_));
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
  bool GetRefinedT_MATCH_QUERY(
      const std::shared_ptr<global_mapping::Submap>& matched_submap,
      const std::shared_ptr<global_mapping::Submap>& query_submap,
      const Eigen::Matrix4d& T_MATCH_QUERY_EST,
      Eigen::Matrix4d& T_MATCH_QUERY_OPT) {
    // extract and filter clouds from match submap
    PointCloudPtr match_cloud_world = std::make_shared<PointCloud>();
    std::vector<PointCloud> match_clouds_world_raw =
        matched_submap->GetLidarPointsInWorldFrame(10e6);
    for (const PointCloud& cloud : match_clouds_world_raw) {
      (*match_cloud_world) += cloud;
    }
    *match_cloud_world =
        beam_filtering::FilterPointCloud(*match_cloud_world, filter_params_);

    // extract and filter clouds from query submap
    PointCloudPtr query_cloud_world = std::make_shared<PointCloud>();
    std::vector<PointCloud> query_clouds_world_raw =
        query_submap->GetLidarPointsInWorldFrame(10e6);
    for (const PointCloud& cloud : match_clouds_world_raw) {
      (*query_cloud_world) += cloud;
    }

    *query_cloud_world =
        beam_filtering::FilterPointCloud(*query_cloud_world, filter_params_);

    // match clouds
    matcher_->SetRef(match_cloud_world);
    matcher_->SetTarget(query_cloud_world);
    if (!matcher_->Match()) {
      BEAM_WARN("Failed scan matching. Not adding loop closure constraint.");
      return false;
    }
    T_MATCH_QUERY_OPT = matcher_->GetResult().inverse().matrix();

    return true;
  }

  std::string matcher_config_;
  std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher_;
  Eigen::Matrix<double, 6, 6> loop_closure_covariance_;
  std::vector<beam_filtering::FilterParamsType> filter_params_;
  std::string source_{"SCANREGLOOPCLOSURE"};
};

using LoopClosureRefinementIcp =
    LoopClosureRefinementScanRegistration<beam_matching::IcpMatcher,
                                          beam_matching::IcpMatcher::Params>;
using LoopClosureRefinementGicp =
    LoopClosureRefinementScanRegistration<beam_matching::GicpMatcher,
                                          beam_matching::GicpMatcher::Params>;
using LoopClosureRefinementNdt =
    LoopClosureRefinementScanRegistration<beam_matching::NdtMatcher,
                                          beam_matching::NdtMatcher::Params>;

}  // namespace loop_closure

}  // namespace bs_models
