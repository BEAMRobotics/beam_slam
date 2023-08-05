#pragma once

#include <list>

#include <beam_matching/Matcher.h>
#include <beam_matching/loam/LoamPointCloud.h>
#include <beam_utils/pointclouds.h>

#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>
#include <bs_models/lidar/scan_pose.h>
#include <bs_models/scan_registration/registration_map.h>
#include <bs_models/scan_registration/scan_registration_base.h>

namespace bs_models { namespace scan_registration {

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
   * @brief Constructor that requires base params
   * @brief base_params
   */
  ScanToMapRegistrationBase(const ScanRegistrationParamsBase& base_params);

  /**
   * @brief delete default constructor
   */
  ScanToMapRegistrationBase() = delete;

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
  bs_constraints::relative_pose::Pose3DStampedTransaction
      RegisterNewScan(const ScanPose& new_scan) override;

protected:
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

  const std::string source_{"SCANTOMAPREGISTRATION"};

  /** This is used for calculating relative pose between scans instead of a
   * global pose for each scan. To create these transactions, we need the actual
   * variables that are in the graph, and the measure transform from map to scan
   * frame.
   *
   * NOTE: This scan pose only contains poses, no scan.
   *
   */
  std::unique_ptr<ScanPose> scan_pose_prev_;
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
    Params(const ScanRegistrationParamsBase& base_params, int _map_size);

    /** number of prev scans to save in the map */
    int map_size{10};

    /** load derived params & base params */
    void LoadFromJson(const std::string& config);

    void Print(std::ostream& stream = std::cout) const;

    /** Get the base class params */
    ScanRegistrationParamsBase GetBaseParams() const;
  };

  ScanToMapLoamRegistration(std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher,
                            const ScanRegistrationParamsBase& base_params,
                            int map_size = 10);

private:
  bool RegisterScanToMap(const ScanPose& scan_pose,
                         Eigen::Matrix4d& T_MAP_SCAN) override;

  void AddScanToMap(const ScanPose& scan_pose,
                    const Eigen::Matrix4d& T_MAP_SCAN) override;

  std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher_;
  Params params_;
};

}} // namespace bs_models::scan_registration
