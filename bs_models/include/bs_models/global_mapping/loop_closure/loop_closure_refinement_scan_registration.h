#pragma once

#include <map>

#include <ros/time.h>
#include <fuse_core/transaction.h>

#include <beam_utils/pointclouds.h>
#include <beam_matching/Matchers.h>
#include <beam_filtering/VoxelDownsample.h>

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
    downsampling_voxel_size_ = J["downsampling_voxel_size"];
    downsampling_max_map_size_ = J["downsampling_max_map_size"];
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
    // setup downsampler
    Eigen::Vector3f scan_voxel_size(downsampling_voxel_size_,
                                    downsampling_voxel_size_,
                                    downsampling_voxel_size_);
    beam_filtering::VoxelDownsample downsampler(scan_voxel_size);

    // extract and filter clouds from match submap
    PointCloudPtr match_cloud_world = std::make_shared<PointCloud>();
    std::vector<PointCloud> match_clouds_world_raw =
        matched_submap.GetLidarPointsInWorldFrame(downsampling_max_map_size_);
    for (const PointCloud& cloud : match_clouds_world_raw) {
      PointCloud downsampled;
      downsampler.Filter(cloud, downsampled);
      (*match_cloud_world) += downsampled;
    }

    // extract and filter clouds from query submap
    PointCloudPtr query_cloud_world = std::make_shared<PointCloud>();
    std::vector<PointCloud> query_clouds_world_raw =
        query_submap.GetLidarPointsInWorldFrame(downsampling_max_map_size_);
    for (const PointCloud& cloud : query_clouds_world_raw) {
      PointCloud downsampled;
      downsampler.Filter(cloud, downsampled);
      (*query_cloud_world) += downsampled;
    }

    // match clouds
    matcher_->SetRef(match_cloud_world);
    matcher_->SetTarget(query_cloud_world);
    if (!matcher_->Match()) {
      BEAM_WARN("Failed scan matching. Not adding loop closure constraint.");
      return false;
    }
    T_MATCH_QUERY_OPT = matcher_->GetResult().inverse().matrix();
  }

  std::unique_ptr<Matcher<PointCloudPtr>> matcher_;

  /** Path to matcher config. If empty, it will use default params. */
  std::string matcher_config_{""};

  /** leaf size for downsampling filter on max map size. If set to zero, it will
   * not downsample. */
  float downsampling_voxel_size_{0.03};

  /** Each submap will be combined into a few pointsclouds, then downsampled,
   * then combined into one pointcloud. This sets the max size of these clouds.
   */
  int downsampling_max_map_size_{500000};

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
