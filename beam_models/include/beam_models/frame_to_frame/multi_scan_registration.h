#pragma once

#include <list>
#include <unordered_set>

#include <beam_matching/Matcher.h>
#include <beam_utils/pointclouds.h>

#include <beam_models/frame_to_frame/scan_pose.h>

namespace beam_models { namespace frame_to_frame {

class MultiScanRegistration {
public:
  MultiScanRegistration(
      std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher,
      int num_neighbors, double outlier_threshold_t, double outlier_threshold_r,
      const std::string& source);

  ~MultiScanRegistration() = default;

  void SetFixedCovariance(const Eigen::Matrix<double, 6, 6>& covariance);

  fuse_core::Transaction::SharedPtr
      RegisterNewScan(const std::shared_ptr<ScanPose>& new_scan);

  void UpdateScanPoses(fuse_core::Graph::ConstSharedPtr graph_msg);

private:
  void MatchScans(const PointCloudPtr& cloud1, const PointCloudPtr& cloud2,
                  const Eigen::Matrix4d& T_WORLD_CLOUD1,
                  const Eigen::Matrix4d& T_WORLD_CLOUD2,
                  Eigen::Matrix4d& T_CLOUD1_CLOUD2,
                  Eigen::Matrix<double, 6, 6>& covariance);

  bool PassedThreshold(const Eigen::Matrix4d& T_measured,
                       const Eigen::Matrix4d& T_estimated);

  std::list<std::shared_ptr<ScanPose>> reference_clouds_;
  std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher_;
  int num_neighbors_;
  double outlier_threshold_t_;
  double outlier_threshold_r_;
  std::string source_;
  Eigen::Matrix<double, 6, 6> covariance_;
  bool use_fixed_covariance_{false};

  // Extra debugging tools: these must be set here, not in the config file
  bool output_scan_registration_results_{true};
  std::string tmp_output_path_{"/home/nick/tmp/"};
};

}} // namespace beam_models::frame_to_frame
