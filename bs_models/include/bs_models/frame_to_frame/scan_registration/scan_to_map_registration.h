#pragma once

#include <list>

#include <beam_matching/Matcher.h>
#include <beam_utils/pointclouds.h>
#include <beam_matching/loam/LoamPointCloud.h>

#include <bs_constraints/frame_to_frame/pose_3d_stamped_transaction.h>
#include <bs_models/frame_to_frame/scan_registration/scan_registration_base.h>
#include <bs_common/scan_pose.h>
#include <bs_common/lidar_map.h>

namespace bs_models {
namespace frame_to_frame {

using namespace beam_matching;
using namespace bs_common;

/**
 * @brief Base class that defines the interface for ScanToMapRegistration
 * classes. Differnt classes may have slightly different implementations based
 * on the point cloud type so a base class here helps remove duplicate code.
 */
class ScanToMapRegistrationBase : public ScanRegistrationBase {
 public:

  /**
   * @brief Constructor that requires initial params
   * @param fix_first_scan set to true to give an almost perfect prior to the
   * first scan pose
   */
  ScanToMapRegistrationBase(bool fix_first_scan);

  /**
   * @brief Default constructor which uses default class parameters
   */
  ~ScanToMapRegistrationBase() = default;

  /**
   * @brief register a new scan and return the transaction generated
   * @param new_scan scan pose to register to the current map
   * @return transaction with constraint between the current scan pose and the
   * previous, unless the map is empty then the transaction will only contain a
   * prior constraint on this pose
   */
  bs_constraints::frame_to_frame::Pose3DStampedTransaction RegisterNewScan(
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
   * @brief Pure virtual function for adding a new scan to a map. This should
   * also trim the map after adding the scan, if required.
   */
  virtual void AddScanToMap(const ScanPose& scan_pose,
                            const Eigen::Matrix4d& T_MAP_SCAN) = 0;

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
};

/**
 * @brief Derived class that implements scan to map registration using loam
 * matching
 */
class ScanToMapLoamRegistration : public ScanToMapRegistrationBase {
 public:
  struct Params : public ScanRegistrationParamsBase {
    Params() = default;

    /** constructor that takes in a base params object */
    Params(const ScanRegistrationParamsBase& base_params, int _map_size,
           bool _store_full_cloud);

    /** number of prev scans to save in the map */
    int map_size{10};

    /** If set to true, it will extract the loam and regular point cloud from
     * the scan poses, even though only the loam clouds are used for
     * registration. The reason for this is that we may want to build a fully
     * dense map for other purposes, but also build a loam map for registration.
     */
    bool store_full_cloud{true};

    /** load derived params & base params */
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

  bool PassedMinMotion(const Eigen::Matrix4d& T_CLOUD1_CLOUD2);

  bool PassedRegThreshold(const Eigen::Matrix4d& T_measured);

  std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher_;
  Params params_;
};

}  // namespace frame_to_frame
}  // namespace bs_models
