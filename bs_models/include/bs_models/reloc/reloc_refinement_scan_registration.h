#pragma once

#include <filesystem>
#include <map>

#include <fuse_core/transaction.h>
#include <ros/time.h>

#include <beam_filtering/Utils.h>
#include <beam_matching/Matchers.h>
#include <beam_utils/pointclouds.h>

#include <bs_common/conversions.h>
#include <bs_common/utils.h>
#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>
#include <bs_models/reloc/reloc_refinement_base.h>
#include <bs_models/scan_registration/scan_registration_base.h>

namespace bs_models { namespace reloc {

/**
 * @brief Templated class for reloc refinement with lidar scan matching
 */
template <typename MatcherType, typename ParamsType>
class RelocRefinementScanRegistration : public RelocRefinementBase {
public:
  /**
   * @brief constructor requiring only a path to a config file
   * @param config full path to config json
   */
  RelocRefinementScanRegistration(const std::string& config)
      : RelocRefinementBase(config) {
    LoadConfig();
    Setup();
  }

  /**
   * @brief see base class
   */
  RelocRefinementResults
      RunRefinement(const global_mapping::SubmapPtr& matched_submap,
                    const global_mapping::SubmapPtr& query_submap,
                    const Eigen::Matrix4d& T_MATCH_QUERY_EST,
                    const std::string& output_path = "") override {
    // extract and filter clouds from matched submap
    PointCloud matched_submap_world = beam_filtering::FilterPointCloud(
        matched_submap->GetLidarPointsInWorldFrameCombined(), filter_params_);
    PointCloud matched_submap_in_submap_frame;
    pcl::transformPointCloud(
        matched_submap_world, matched_submap_in_submap_frame,
        beam::InvertTransform(matched_submap->T_WORLD_SUBMAP()));

    // extract and filter clouds from matched submap
    PointCloud query_submap_world = beam_filtering::FilterPointCloud(
        query_submap->GetLidarPointsInWorldFrameCombined(), filter_params_);

    PointCloud query_submap_in_submap_frame;
    pcl::transformPointCloud(
        query_submap_world, query_submap_in_submap_frame,
        beam::InvertTransform(query_submap->T_WORLD_SUBMAP()));

    std::string current_output_path;
    if (!output_path.empty()) {
      if (!std::filesystem::exists(output_path)) {
        BEAM_ERROR("Output path for RelocRefinement does not exist: {}",
                   output_path);
        throw std::runtime_error{"invalid output path"};
      }
      std::string query_stamp = std::to_string(query_submap->Stamp().toSec());
      current_output_path = beam::CombinePaths(output_path, query_stamp);

      BEAM_INFO("Saving reloc refinement results to: {}", current_output_path);
      std::filesystem::create_directory(current_output_path);
    }

    // get refined transform
    RelocRefinementResults results;
    results.successful = GetRefinedT_SUBMAP_QUERY(
        matched_submap_in_submap_frame, query_submap_in_submap_frame,
        T_MATCH_QUERY_EST, current_output_path, results.T_MATCH_QUERY);
    return results;
  }

private:
  /**
   * @brief Method for loading a config json file.
   */
  void LoadConfig() {
    std::string read_path = config_path_;
    if (read_path.empty()) {
      BEAM_INFO("no config path provided, using default params");
      return;
    }

    BEAM_INFO("Loading reloc config: {}", read_path);
    nlohmann::json J;
    if (!beam::ReadJson(read_path, J)) {
      BEAM_INFO("Unable to read reloc refinement scan registration params.");
      throw std::runtime_error{"Unable to read json"};
      return;
    }

    beam::ValidateJsonKeysOrThrow(
        std::vector<std::string>{"matcher_config", "filters"}, J);
    std::string matcher_config_rel = J["matcher_config"];
    if (matcher_config_rel.empty()) {
      BEAM_ERROR("Reloc refinement cannot have an empty matcher_config");
      throw std::runtime_error{"invalid json inputs"};
    }
    matcher_config_ = beam::CombinePaths(bs_common::GetBeamSlamConfigPath(),
                                         matcher_config_rel);

    nlohmann::json J_filters = J["filters"];

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
   * @param submap_cloud submap cloud in submap frame
   * @param query_cloud either a scan to register against a submap, or a query
   * submap cloud to match against some other submap. This is in the query frame
   * submaps
   * @param T_SUBMAP_QUERY_EST estimated transform between the query cloud and
   * the submap. This usually is determined with a class derived from
   * RelocCandidateSearchBase
   * @param output_path will save results if not empty
   * @param T_SUBMAP_QUERY_OPT reference to the resulting refined transform from
   * query scan to the submap in question
   */
  bool GetRefinedT_SUBMAP_QUERY(const PointCloud& submap_cloud,
                                const PointCloud& query_cloud,
                                const Eigen::Matrix4d& T_SUBMAP_QUERY_EST,
                                const std::string& output_path,
                                Eigen::Matrix4d& T_SUBMAP_QUERY_OPT) {
    // convert query to estimated submap frame and make pointers
    PointCloudPtr submap_ptr = std::make_shared<PointCloud>(submap_cloud);
    PointCloudPtr query_in_submap_frame_est = std::make_shared<PointCloud>();
    pcl::transformPointCloud(query_cloud, *query_in_submap_frame_est,
                             T_SUBMAP_QUERY_EST);

    // match clouds
    matcher_->SetRef(submap_ptr);
    matcher_->SetTarget(query_in_submap_frame_est);

    if (!matcher_->Match()) {
      BEAM_WARN("Failed scan matching. Not adding reloc constraint.");
      if (!output_path.empty()) {
        matcher_->SaveResults(output_path, "failed_cloud_");
      }
      return false;
    }

    T_SUBMAP_QUERY_OPT = matcher_->ApplyResult(T_SUBMAP_QUERY_EST);
    if (!output_path.empty()) { matcher_->SaveResults(output_path, "cloud_"); }

    return true;
  }

  std::string matcher_config_;
  std::unique_ptr<PointcloudMatcher> matcher_;
  std::vector<beam_filtering::FilterParamsType> filter_params_;
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

}} // namespace bs_models::reloc
