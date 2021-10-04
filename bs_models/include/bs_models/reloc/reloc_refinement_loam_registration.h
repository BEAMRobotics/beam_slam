#pragma once

#include <map>

#include <ros/time.h>
#include <fuse_core/transaction.h>
#include <nlohmann/json.hpp>

#include <beam_utils/pointclouds.h>
#include <beam_matching/LoamMatcher.h>
#include <bs_models/reloc/reloc_refinement_base.h>
#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>

namespace bs_models {

namespace reloc {

/**
 * @brief reloc refinement with loam scan matching
 */
class RelocRefinementLoam : public RelocRefinementBase {
 public:
  /**
   * @brief Constructor that only takes in a config path and covariance matrix
   * @param config full path to config json
   * @param reloc_covariance
   */
  RelocRefinementLoam(const Eigen::Matrix<double, 6, 6>& reloc_covariance,
                      const std::string& config = "");

  /**
   * @brief Generate a fuse transaction between two candidate loop closure
   * submaps
   * @param matched_submap submap that a new query submap matches to
   * @param query_submap new submap that we are adding constraints with previous
   * submaps
   * @param T_MATCH_QUERY_EST estimated transform between the two submaps. This
   * is determined with a class derived from RelocCandidateSearchBase
   */
  fuse_core::Transaction::SharedPtr GenerateTransaction(
      const SubmapPtr& matched_submap, const SubmapPtr& query_submap,
      const Eigen::Matrix4d& T_MATCH_QUERY_EST) override;

  /**
   * @brief Implements the pure virtual function defined in the base class which
   * gets a refined pose from a candidate submap, an initial transform and some
   * lidar + camera data
   * @param T_SUBMAP_QUERY_refined reference to tranform from query pose
   * (baselink) to the submap
   * @param T_SUBMAP_QUERY_initial initial guess of transform from query pose
   * (baselink) to submap
   * @param submap submap that we think the query pose is inside
   * @param lidar_cloud_in_query_frame not used in this implementation
   * @param loam_cloud_in_query_frame
   * @param image not used in this implementation
   * @return true if successful, false otherwise
   */
  bool GetRefinedPose(Eigen::Matrix4d& T_SUBMAP_QUERY_refined,
                      const Eigen::Matrix4d& T_SUBMAP_QUERY_initial,
                      const SubmapPtr& submap,
                      const PointCloud& lidar_cloud_in_query_frame,
                      const LoamPointCloudPtr& loam_cloud_in_query_frame,
                      const cv::Mat& image = cv::Mat()) override;

 private:
  /**
   * @brief method for loading a config json file.
   */
  void LoadConfig();

  /**
   * @param takes all params loaded from LoadConfig, and initializes all member
   * classes/variables
   */
  void Setup();

  /**
   * @brief Calculate a refined pose between submaps using loam registration
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
  bool GetRefinedT_SUBMAP_QUERY(const LoamPointCloud& submap_cloud,
                                const LoamPointCloud& query_cloud,
                                const Eigen::Matrix4d& T_SUBMAP_QUERY_EST,
                                Eigen::Matrix4d& T_SUBMAP_QUERY_OPT);

  std::string matcher_config_;
  std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher_;
  Eigen::Matrix<double, 6, 6> reloc_covariance_;
  std::string source_{"LOAMSCANREGRELOC"};
};

}  // namespace reloc

}  // namespace bs_models
