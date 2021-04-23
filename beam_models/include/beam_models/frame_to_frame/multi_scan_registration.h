#pragma once

#include <list>
#include <unordered_set>

#include <beam_matching/Matcher.h>
#include <beam_utils/pointclouds.h>

#include <beam_constraints/frame_to_frame/pose_3d_stamped_transaction.h>
#include <beam_models/frame_to_frame/scan_pose.h>

static bool tmp_{true};

namespace beam_models { namespace frame_to_frame {

template <typename ConstraintType, typename PriorType>
using TransactionBase =
    beam_constraints::frame_to_frame::FrameToFrameTransactionBase<
        ConstraintType, PriorType>;

class MultiScanRegistration {
public:
  struct Params {
    int num_neighbors;
    double outlier_threshold_t;
    double outlier_threshold_r;
    std::string source{"MULTISCANREGISTRATION"};
    double lag_duration{0};
    bool fix_first_scan{false};
  };

  MultiScanRegistration(
      std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher,
      const Params& params);

  ~MultiScanRegistration() = default;

  void SetFixedCovariance(const Eigen::Matrix<double, 6, 6>& covariance);

  beam_constraints::frame_to_frame::Pose3DStampedTransaction
      RegisterNewScan(const ScanPose& new_scan);

  void UpdateScanPoses(fuse_core::Graph::ConstSharedPtr graph_msg);

  void RemoveOldScans(const ros::Time& new_scan_time);

  void RemoveMissingScans(fuse_core::Graph::ConstSharedPtr graph_msg,
                          bool require_one_update = true);

  inline std::list<ScanPose>::iterator Begin() {
    return reference_clouds_.begin();
  }

  inline std::list<ScanPose>::iterator End() { return reference_clouds_.end(); }

  inline int GetNumStoredScans() { return reference_clouds_.size(); }

  ScanPose GetScan(const ros::Time& t, bool& success = tmp_);

  void PrintScanDetails(std::ostream& stream = std::cout);

private:
  bool MatchScans(const ScanPose& scan_pose_1, const ScanPose& scan_pose_2,
                  Eigen::Matrix4d& T_CLOUD1_CLOUD2,
                  Eigen::Matrix<double, 6, 6>& covariance);

  bool PassedThreshold(const Eigen::Matrix4d& T_measured,
                       const Eigen::Matrix4d& T_estimated);

  std::list<ScanPose> reference_clouds_;

  std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher_;
  Params params_;
  Eigen::Matrix<double, 6, 6> covariance_;
  bool use_fixed_covariance_{false};
  double pose_prior_noise_{1e-9};

  // Extra debugging tools: these must be set here, not in the config file
  bool output_scan_registration_results_{false};
  std::string current_scan_path_;
  std::string tmp_output_path_{
      "/home/nick/results/beam_slam/scan_registration/"};
  PointCloudColPtr coord_frame_;
};

}} // namespace beam_models::frame_to_frame
