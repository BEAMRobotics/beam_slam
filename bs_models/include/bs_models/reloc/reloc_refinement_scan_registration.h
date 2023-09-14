#pragma once

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
  fuse_core::Transaction::SharedPtr
      GenerateTransaction(const global_mapping::SubmapPtr& matched_submap,
                          const global_mapping::SubmapPtr& query_submap,
                          const Eigen::Matrix4d& T_MATCH_QUERY_EST) override {
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

    // get refined transform
    Eigen::Matrix4d T_MATCH_QUERY_OPT;
    if (!GetRefinedT_SUBMAP_QUERY(matched_submap_in_submap_frame,
                                  query_submap_in_submap_frame,
                                  T_MATCH_QUERY_EST, T_MATCH_QUERY_OPT)) {
      return nullptr;
    }

    // create transaction
    bs_constraints::Pose3DStampedTransaction transaction(query_submap->Stamp());
    transaction.AddPoseConstraint(
        matched_submap->Position(), query_submap->Position(),
        matched_submap->Orientation(), query_submap->Orientation(),
        bs_common::TransformMatrixToVectorWithQuaternion(T_MATCH_QUERY_OPT),
        reloc_covariance_, source_);

    return transaction.GetTransaction();
  }

  /**
   * @brief Implements the pure virtual function defined in the base class which
   * gets a refined pose from a candidate submap, an initial transform and some
   * lidar + camera data
   * @param T_SUBMAP_QUERY_refined reference to tranform from query pose
   * (baselink) to the submap
   * @param T_SUBMAP_QUERY_initial initial guess of transform from query pose
   * (baselink) to submap
   * @param submap submap that we think the query pose is inside
   * @param lidar_cloud_in_query_frame
   * @param loam_cloud_in_query_frame not used in this implementation
   * @return true if successful, false otherwise
   */
  bool GetRefinedPose(Eigen::Matrix4d& T_SUBMAP_QUERY_refined,
                      const Eigen::Matrix4d& T_SUBMAP_QUERY_initial,
                      const global_mapping::SubmapPtr& submap,
                      const PointCloud& lidar_cloud_in_query_frame,
                      const beam_matching::LoamPointCloudPtr&
                          loam_cloud_in_query_frame = nullptr) override {
    // extract and filter clouds from match submap
    PointCloud submap_cloud_world = beam_filtering::FilterPointCloud(
        submap->GetLidarPointsInWorldFrameCombined(), filter_params_);
    PointCloud submap_in_submap_frame;
    pcl::transformPointCloud(submap_cloud_world, submap_in_submap_frame,
                             beam::InvertTransform(submap->T_WORLD_SUBMAP()));

    // get refined transform
    if (!GetRefinedT_SUBMAP_QUERY(
            submap_in_submap_frame, lidar_cloud_in_query_frame,
            T_SUBMAP_QUERY_initial, T_SUBMAP_QUERY_refined)) {
      return false;
    }

    return true;
  }

private:
  /**
   * @brief Method for loading a config json file.
   */
  void LoadConfig() {
    std::string read_path = config_path_;
    if (read_path.empty()) { return; }

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
      BEAM_ERROR("Missing one or more parameter, using default reloc "
                 "refinement params");
    }

    nlohmann::json J_filters;
    try {
      J_filters = J["filters"];
    } catch (...) {
      ROS_ERROR("Missing 'filters' param in reloc config file. Not using "
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
   * @param submap_cloud submap cloud in submap frame
   * @param query_cloud either a scan to register against a submap, or a query
   * submap cloud to match against some other submap. This is in the query frame
   * submaps
   * @param T_SUBMAP_QUERY_EST estimated transform between the query cloud and
   * the submap. This usually is determined with a class derived from
   * RelocCandidateSearchBase
   * @param T_SUBMAP_QUERY_OPT reference to the resulting refined transform from
   * query scan to the submap in question
   */
  bool GetRefinedT_SUBMAP_QUERY(const PointCloud& submap_cloud,
                                const PointCloud& query_cloud,
                                const Eigen::Matrix4d& T_SUBMAP_QUERY_EST,
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

    Eigen::Matrix4d T_SUBMAPREFINED_SUBMAPEST =
        matcher_->GetResult().inverse().matrix();

    /**
     * Get refined pose:
     * T_SUBMAP_QUERY_OPT = T_SUBMAPREFINED_QUERY
     *                   = T_SUBMAPREFINED_SUBMAPEST * T_SUBMAPEST_QUERY
     *                   = T_SUBMAPREFINED_SUBMAPEST * T_SUBMAP_QUERY_EST
     */
    T_SUBMAP_QUERY_OPT = T_SUBMAPREFINED_SUBMAPEST * T_SUBMAP_QUERY_EST;

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
  std::unique_ptr<PointcloudMatcher> matcher_;
  Eigen::Matrix<double, 6, 6> reloc_covariance_;
  std::vector<beam_filtering::FilterParamsType> filter_params_;
  std::string source_{"RelocRefinementScanRegistration"};
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
