#pragma once

#include <map>

#include <ros/time.h>
#include <fuse_core/transaction.h>

#include <beam_utils/pointclouds.h>
#include <beam_matching/Matchers.h>
#include <beam_filtering/Utils.h>

#include <bs_models/reloc/reloc_refinement_base.h>
#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>
#include <bs_common/utils.h>

namespace bs_models {

namespace reloc {

/**
 * @brief Templated class for reloc refinement with lidar scan matching
 */
template <typename MatcherType, typename ParamsType>
class RelocRefinementScanRegistration : public RelocRefinementBase {
 public:
  /**
   * @brief constructor requiring only a path to a config file
   * @param config full path to config json
   * @param reloc_covariance
   */
  RelocRefinementScanRegistration(
      const Eigen::Matrix<double, 6, 6>& reloc_covariance,
      const std::string& config = "")
      : RelocRefinementBase(config), reloc_covariance_(reloc_covariance) {
    LoadConfig();
    Setup();
  }

  /**
   * @brief Generate a fuse transaction between two candidate reloc
   * submaps
   * @param matched_submap submap that a new query submap matches to
   * @param query_submap new submap that we are adding constraints with previous
   * submaps
   * @param T_MATCH_QUERY_EST estimated transform between the two submaps. This
   * is determined with a class derived from RelocCandidateSearchBase
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
        reloc_covariance_, source_);

    return transaction.GetTransaction();
  }

  /**
   * @brief Overrides function that gets a refined pose from a candidate
   * submap and an initial transform
   * @param T_SUBMAP_QUERY_refined reference to tranform from query pose
   * (baselink) to the submap
   * @param T_SUBMAP_QUERY_initial initial guess of transform from query pose
   * (baselink) to submap
   * @param submap submap that we think the query pose is inside
   * @param lidar_cloud_in_query_frame
   * @param submap_msg reference to submap msg to fill
   * @return true if successful
   */
  bool GetRefinedPose(Eigen::Matrix4d& T_SUBMAP_QUERY_refined,
                      const Eigen::Matrix4d& T_SUBMAP_QUERY_initial,
                      const std::shared_ptr<global_mapping::Submap>& submap,
                      const PointCloud& lidar_cloud_in_query_frame,
                      const cv::Mat& image = cv::Mat()) override {
    // TODO
    BEAM_ERROR("NOT YET IMPLEMENTED");
  }

 private:
  /**
   * @brief Method for loading a config json file.
   */
  void LoadConfig() {
    std::string read_path = config_path_;
    if (read_path.empty()) {
      return;
    }

    if (read_path == "DEFAULT_PATH") {
      read_path = bs_common::GetBeamSlamConfigPath() +
                  "global_map/reloc_refinement_scan_registration.json";
    }

    BEAM_INFO("Loading reloc config: {}", read_path);
    nlohmann::json J;
    if (!beam::ReadJson(read_path, J)) {
      BEAM_INFO("Using default reloc refinement scan registration params.");
      return;
    }

    try {
      matcher_config_ = J["matcher_config"];
    } catch (...) {
      BEAM_ERROR(
          "Missing one or more parameter, using default reloc "
          "refinement params");
    }

    nlohmann::json J_filters;
    try {
      J_filters = J["filters"];
    } catch (...) {
      ROS_ERROR(
          "Missing 'filters' param in reloc config file. Not using "
          "filters.");
      return;
    }
    filter_params_ = beam_filtering::LoadFilterParamsVector(J_filters);
    BEAM_INFO("Loaded {} input filters", filter_params_.size());
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
   * is determined with a class derived from RelocCandidateSearchBase
   * @param T_MATCH_QUERY_OPT reference to the resulting refined transform from
   * query submap to matched submap
   */
  bool GetRefinedT_MATCH_QUERY(
      const std::shared_ptr<global_mapping::Submap>& matched_submap,
      const std::shared_ptr<global_mapping::Submap>& query_submap,
      const Eigen::Matrix4d& T_MATCH_QUERY_EST,
      Eigen::Matrix4d& T_MATCH_QUERY_OPT) {
    // extract and filter clouds from match submap
    PointCloudPtr match_cloud_world = std::make_shared<PointCloud>(
        matched_submap->GetLidarPointsInWorldFrameCombined());
    *match_cloud_world =
        beam_filtering::FilterPointCloud(*match_cloud_world, filter_params_);

    // extract and filter clouds from query submap
    PointCloudPtr query_cloud_world = std::make_shared<PointCloud>(
        query_submap->GetLidarPointsInWorldFrameCombined());
    *query_cloud_world =
        beam_filtering::FilterPointCloud(*query_cloud_world, filter_params_);

    // match clouds
    matcher_->SetRef(match_cloud_world);
    matcher_->SetTarget(query_cloud_world);

    if (!matcher_->Match()) {
      BEAM_WARN("Failed scan matching. Not adding reloc constraint.");
      if (output_results_) {
        output_path_stamped_ =
            debug_output_path_ +
            beam::ConvertTimeToDate(std::chrono::system_clock::now()) +
            "_failed/";
        boost::filesystem::create_directory(output_path_stamped_);
        matcher_->SaveResults(output_path_stamped_);
      }
      return false;
    }

    Eigen::Matrix4d T_MATCHREFINED_MATCHEST =
        matcher_->GetResult().inverse().matrix();

    /**
     * Get refined pose:
     * T_MATCH_QUERY_OPT = T_MATCHREFINED_QUERY
     *                   = T_MATCHREFINED_MATCHEST * T_MATCHEST_QUERY
     *                   = T_MATCHREFINED_MATCHEST * T_MATCH_QUERY_EST
     */
    T_MATCH_QUERY_OPT = T_MATCHREFINED_MATCHEST * T_MATCH_QUERY_EST;

    if (output_results_) {
      output_path_stamped_ =
          debug_output_path_ +
          beam::ConvertTimeToDate(std::chrono::system_clock::now()) +
          "_passed/";
      boost::filesystem::create_directory(output_path_stamped_);
      matcher_->SaveResults(output_path_stamped_);
    }

    return true;
  }

  std::string matcher_config_;
  std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher_;
  Eigen::Matrix<double, 6, 6> reloc_covariance_;
  std::vector<beam_filtering::FilterParamsType> filter_params_;
  std::string source_{"SCANREGRELOC"};
};

using RelocRefinementIcp =
    RelocRefinementScanRegistration<beam_matching::IcpMatcher,
                                    beam_matching::IcpMatcher::Params>;
using RelocRefinementGicp =
    RelocRefinementScanRegistration<beam_matching::GicpMatcher,
                                    beam_matching::GicpMatcher::Params>;
using RelocRefinementNdt =
    RelocRefinementScanRegistration<beam_matching::NdtMatcher,
                                    beam_matching::NdtMatcher::Params>;

}  // namespace reloc

}  // namespace bs_models
