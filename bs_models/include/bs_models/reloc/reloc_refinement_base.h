#pragma once

#include <map>

#include <ros/time.h>
#include <fuse_core/transaction.h>

#include <beam_utils/pointclouds.h>
#include <bs_models/global_mapping/submap.h>
#include <bs_common/SubmapMsg.h>
#include <bs_common/RelocRequestMsg.h>

namespace bs_models {

namespace reloc {

/**
 * @brief A reloc refinement step that takes an estimated pose
 * from the candidate search and refines the relative pose between the two
 * candidate locations
 */
class RelocRefinementBase {
 public:
  /**
   * @brief constructor with an optional path to a json config
   * @param config path to json config file. If empty, it will use default
   * parameters
   */
  RelocRefinementBase(const std::string& config = "") : config_path_(config){};

  /**
   * @brief default destructor
   */
  ~RelocRefinementBase() = default;

  /**
   * @brief Pure virtual function that generate a fuse transaction between two
   * candidate reloc submaps. This is useful for when using reloc for loop
   * closure.
   * @param matched_submap
   * @param query_submap
   */
  virtual fuse_core::Transaction::SharedPtr GenerateTransaction(
      const std::shared_ptr<global_mapping::Submap>& matched_submap,
      const std::shared_ptr<global_mapping::Submap>& query_submap,
      const Eigen::Matrix4d& T_MATCH_QUERY_EST) = 0;

  /**
   * @brief Pure virtual function that gets a refined pose from a candidate
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
  virtual bool GetRefinedPose(
      Eigen::Matrix4d& T_SUBMAP_QUERY_refined,
      const Eigen::Matrix4d& T_SUBMAP_QUERY_initial,
      const std::shared_ptr<global_mapping::Submap>& submap,
      const PointCloud& lidar_cloud_in_query_frame,
      const cv::Mat& image = cv::Mat()) = 0;

 protected:
  std::string config_path_;

  /* Debugging tools that can only be set here */
  bool output_results_{false};
  std::string debug_output_path_{"/home/nick/results/beam_slam/reloc/"};
  std::string output_path_stamped_;  // to be created in implementation
};

}  // namespace reloc

}  // namespace bs_models
