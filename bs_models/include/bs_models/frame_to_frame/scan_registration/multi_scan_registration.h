#pragma once

#include <list>
#include <unordered_set>

#include <beam_matching/Matcher.h>
#include <beam_utils/pointclouds.h>
#include <beam_matching/loam/LoamPointCloud.h>

#include <bs_constraints/frame_to_frame/pose_3d_stamped_transaction.h>
#include <bs_models/frame_to_frame/scan_registration/scan_registration_base.h>
#include <bs_common/scan_pose.h>

static bool tmp_{true};

namespace bs_models {
namespace frame_to_frame {

using namespace beam_matching;
using namespace bs_common;

class MultiScanRegistrationBase : public ScanRegistrationBase {
 public:
  struct Params : public ScanRegistrationParamsBase {
    Params() = default;

    /** constructor that takes in a base params object */
    Params(const ScanRegistrationParamsBase& base_params, int _num_neighbors,
           double _lag_duration, bool _disable_lidar_map);

    /** number of neibouring scans to register against */
    int num_neighbors{5};

    /** this is needed to know when to remove old scans that have been factored
     * out of the graph. Note that this must be input from the client code, it
     * cannot be set by a json config. The reason for this is because this
     * parameter should come from the yaml file used for the main fuse optimizer
     * config. */
    double lag_duration{0};

    /** Set this to true if you don't want to build a lidar map */
    bool disable_lidar_map{false};

    /** load derived params & base params */
    void LoadFromJson(const std::string& config);

    /** Get the base class params */
    ScanRegistrationParamsBase GetBaseParams();
  };

  MultiScanRegistrationBase(
      const ScanRegistrationParamsBase& base_params, int num_neighbors = 10,
      double lag_duration = 0, bool disable_lidar_map = false);

  MultiScanRegistrationBase() = delete;

  ~MultiScanRegistrationBase() = default;

  bs_constraints::frame_to_frame::Pose3DStampedTransaction RegisterNewScan(
      const ScanPose& new_scan) override;

  // The following public functions are not in the RegistrationBase class so
  // they will not be accessible in client code that uses a pointer to the base
  // class. These can be used for testing, or when instantiating this derived
  // class explicitly

  ScanPose GetScan(const ros::Time& t, bool& success = tmp_);

  void UpdateScanPoses(fuse_core::Graph::ConstSharedPtr graph_msg);

  void RemoveMissingScans(fuse_core::Graph::ConstSharedPtr graph_msg,
                          bool require_one_update = true);

 protected:
  /**
   * @brief pure virtual function that must be overridden in each derived multi
   * scan registraion classes
   */
  virtual bool MatchScans(const ScanPose& scan_pose_ref,
                          const ScanPose& scan_pose_tgt,
                          Eigen::Matrix4d& T_LIDARREF_LIDARTGT,
                          Eigen::Matrix<double, 6, 6>& covariance) = 0;

  void RemoveOldScans(const ros::Time& new_scan_time);

  inline std::list<ScanPose>::iterator Begin() {
    return reference_clouds_.begin();
  }

  inline std::list<ScanPose>::iterator End() { return reference_clouds_.end(); }

  inline int GetNumStoredScans() { return reference_clouds_.size(); }

  void PrintScanDetails(std::ostream& stream = std::cout);

  // Output results to world frame (estimated world frame from the ref cloud)
  void OutputResults(const ScanPose& scan_pose_ref,
                     const ScanPose& scan_pose_tgt,
                     const Eigen::Matrix4d& T_LIDARREF_LIDARTGT,
                     bool output_loam_cloud = false);

  std::list<ScanPose> reference_clouds_;
  Params params_;

  const std::string source_{"MULTISCANREGISTRATION"};

  // Extra debugging tools: these must be set here, not in the config file
  bool output_scan_registration_results_{false};
  std::string current_scan_path_;
  std::string tmp_output_path_{
      "/home/nick/results/beam_slam/scan_registration/multi_scan/"};
  PointCloudCol coord_frame_;
};

class MultiScanLoamRegistration : public MultiScanRegistrationBase {
 public:
  using Params = MultiScanRegistrationBase::Params;

  MultiScanLoamRegistration(
      std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher,
      const ScanRegistrationParamsBase& base_params, int num_neighbors = 10,
      double lag_duration = 0, bool disable_lidar_map = false);

 private:
  bool MatchScans(const ScanPose& scan_pose_ref, const ScanPose& scan_pose_tgt,
                  Eigen::Matrix4d& T_LIDARREF_LIDARTGT,
                  Eigen::Matrix<double, 6, 6>& covariance) override;

  std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher_;
};

class MultiScanRegistration : public MultiScanRegistrationBase {
 public:
  using Params = MultiScanRegistrationBase::Params;

  MultiScanRegistration(
      std::unique_ptr<Matcher<PointCloudPtr>> matcher,
      const ScanRegistrationParamsBase& base_params, int num_neighbors = 10,
      double lag_duration = 0, bool disable_lidar_map = false);

 private:
  bool MatchScans(const ScanPose& scan_pose_ref, const ScanPose& scan_pose_tgt,
                  Eigen::Matrix4d& T_LIDARREF_LIDARTGT,
                  Eigen::Matrix<double, 6, 6>& covariance) override;

  std::unique_ptr<Matcher<PointCloudPtr>> matcher_;
};

}  // namespace frame_to_frame
}  // namespace bs_models
