#pragma once

#include <map>

#include <fuse_core/transaction.h>
#include <ros/time.h>

#include <beam_utils/pointclouds.h>
#include <bs_common/RelocRequestMsg.h>
#include <bs_common/SubmapMsg.h>
#include <bs_models/global_mapping/submap.h>

namespace bs_models::reloc {

/**
 * @brief A reloc refinement step that takes an estimated pose
 * from the candidate search and refines the relative pose between the two
 * candidate locations. There are two required implementations for this class.
 * The first it GenerateTransaction which generates a loop closure transaction
 * (relative pose error) given two submaps. The second required implementation
 * is GetRefinedPose which takes in a submap and some camera + lidar data of the
 * query pose to refine relative to the submap.
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
  virtual fuse_core::Transaction::SharedPtr
      GenerateTransaction(const global_mapping::SubmapPtr& matched_submap,
                          const global_mapping::SubmapPtr& query_submap,
                          const Eigen::Matrix4d& T_MATCH_QUERY_EST) = 0;

  /**
   * @brief Pure virtual function that gets a refined pose from a candidate
   * submap, an initial transform and some lidar + camera data
   * @param T_SUBMAP_QUERY_refined reference to tranform from query pose
   * (baselink) to the submap
   * @param T_SUBMAP_QUERY_initial initial guess of transform from query pose
   * (baselink) to submap
   * @param submap submap that we think the query pose is inside
   * @param lidar_cloud_in_query_frame
   * @param loam_cloud_in_query_frame
   * @param image
   * @return true if successful, false otherwise
   */
  virtual bool GetRefinedPose(
      Eigen::Matrix4d& T_SUBMAP_QUERY_refined,
      const Eigen::Matrix4d& T_SUBMAP_QUERY_initial,
      const global_mapping::SubmapPtr& submap,
      const PointCloud& lidar_cloud_in_query_frame,
      const beam_matching::LoamPointCloudPtr& loam_cloud_in_query_frame,
      const cv::Mat& image) = 0;

protected:
  std::string config_path_;

  /* Debugging tools that can only be set here */
  bool output_results_{false};
  std::string debug_output_path_{"/userhome/debug/reloc/"};
  std::string output_path_stamped_; // to be created in implementation
};

} // namespace bs_models::reloc
