#pragma once

#include <list>

#include <beam_matching/Matcher.h>
#include <beam_utils/pointclouds.h>
#include <beam_matching/loam/LoamPointCloud.h>

#include <beam_constraints/frame_to_frame/pose_3d_stamped_transaction.h>
#include <beam_models/frame_to_frame/scan_registration/scan_registration_base.h>
#include <beam_common/scan_pose.h>

namespace beam_models {
namespace frame_to_frame {

using namespace beam_matching;
using namespace beam_common;

class ScanToMapRegistrationBase : public ScanRegistrationBase {
 public:
  ScanToMapRegistrationBase(bool fix_first_scan);

  ~ScanToMapRegistrationBase() = default;

  beam_constraints::frame_to_frame::Pose3DStampedTransaction RegisterNewScan(
      const ScanPose& new_scan) override;

 protected:
  /**
   * @brief Pure virtual function for determining if a map is empty. This is
   * used to determine whether or not we register a scan then add it, or just
   * add it.
   */
  virtual bool IsMapEmpty() = 0;

  /**
   * @brief Pure virtual function for registering a new scan to the map.
   */
  virtual bool RegisterScanToMap(const ScanPose& scan_pose,
                                 Eigen::Matrix4d& T_MAP_SCAN) = 0;

  /**
   * @brief Pure virtual function for adding a new scan to a map.
   */
  virtual void AddScanToMap(const ScanPose& scan_pose,
                            const Eigen::Matrix4d& T_MAP_SCAN) = 0;

  /**
   * @brief Pure virtual function for trimming the map.
   */
  virtual void TrimMap(const Eigen::Matrix4d& T_MAP_SCAN) = 0;

  double pose_prior_noise_{1e-9};
  bool fix_first_scan_{true};
  const std::string source_{"SCANTOMAPREGISTRATION"};

  /** These are used for calculating relative pose between scans instead of a
   * global pose for each scan. To create these transactions, we need the actual
   * variables that are in the graph, and the measure transform from map to scan
   * frame. */
  fuse_variables::Position3DStamped scan_prev_position_;
  fuse_variables::Orientation3DStamped scan_prev_orientation_;
  Eigen::Matrix4d T_MAP_SCANPREV_;

  // Extra debugging tools: these must be set here, not in the config file
  bool output_scan_registration_results_{false};
  std::string current_scan_path_;
  std::string tmp_output_path_{
      "/home/nick/results/beam_slam/scan_registration/"};
  PointCloudCol coord_frame_;
};

class ScanToMapLoamRegistration : public ScanToMapRegistrationBase {
 public:
  struct Params {
    double outlier_threshold_t{0.3};
    double outlier_threshold_r{20};
    double min_motion_trans_m{0};
    double min_motion_rot_rad{0};
    std::string source{"SCANTOMAPREGISTRATION"};
    bool fix_first_scan{false};
    int map_size{10};

    void LoadFromJson(const std::string& config);
  };

  ScanToMapLoamRegistration(std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher,
                            const Params& params);

 private:
  bool IsMapEmpty() override;

  bool RegisterScanToMap(const ScanPose& scan_pose,
                         Eigen::Matrix4d& T_MAP_SCAN) override;

  void AddScanToMap(const ScanPose& scan_pose,
                    const Eigen::Matrix4d& T_MAP_SCAN) override;

  void TrimMap(const Eigen::Matrix4d& T_MAP_SCAN) override;

  bool PassedMinMotion(const Eigen::Matrix4d& T_CLOUD1_CLOUD2);

  bool PassedRegThreshold(const Eigen::Matrix4d& T_measured);

  std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher_;
  std::list<LoamPointCloud> clouds_in_map_frame_;
  Params params_;
};

}  // namespace frame_to_frame
}  // namespace beam_models
