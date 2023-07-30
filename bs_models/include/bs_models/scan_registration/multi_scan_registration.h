#pragma once

#include <list>
#include <unordered_set>

#include <beam_matching/Matcher.h>
#include <beam_matching/loam/LoamPointCloud.h>
#include <beam_utils/pointclouds.h>

#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>
#include <bs_models/lidar/scan_pose.h>
#include <bs_models/scan_registration/scan_registration_base.h>

static bool _tmp_bool{true};

namespace bs_models { namespace scan_registration {

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

  MultiScanRegistrationBase(const ScanRegistrationParamsBase& base_params,
                            int num_neighbors = 10, double lag_duration = 0,
                            bool disable_lidar_map = false);

  MultiScanRegistrationBase() = delete;

  ~MultiScanRegistrationBase() = default;

  bs_constraints::relative_pose::Pose3DStampedTransaction
      RegisterNewScan(const ScanPose& new_scan) override;

  // The following public functions are not in the RegistrationBase class so
  // they will not be accessible in client code that uses a pointer to the base
  // class. These can be used for testing, or when instantiating this derived
  // class explicitly

  ScanPose GetScan(const ros::Time& t, bool& success = _tmp_bool);

  void UpdateScanPoses(fuse_core::Graph::ConstSharedPtr graph_msg);

  void RemoveMissingScans(fuse_core::Graph::ConstSharedPtr graph_msg,
                          bool require_one_update = true);

  inline MultiScanRegistrationBase::Params GetParams() const { return params_; }

protected:
  /**
   * @brief Add scan to lidar map, if not disabled, and add prior to the
   * transaction
   * @param scan new scan pose to add to map and get prior for
   * @param transaction reference to a transaction for adding a prior
   */
  void AddFirstScan(
      const ScanPose& scan,
      bs_constraints::relative_pose::Pose3DStampedTransaction& transaction);

  /**
   * @brief Add scan to list of reference scans, while keeping times sorted. It
   * also clears the oldest scans if the list is larger than the max allowable
   * @param scan new scan pose to add
   */
  void InsertCloudInReferences(const ScanPose& scan);

  /**
   * @brief Registers a scan pose to all scan poses stored in the reference
   * scans list, and adds the constraints to the transaction. Note this does not
   * add variables to the transaction, this is managed outside the function.
   */
  int RegisterScanToReferences(
      const ScanPose& new_scan,
      bs_constraints::relative_pose::Pose3DStampedTransaction& transaction);

  /**
   * @brief pure virtual function that must be overridden in each derived multi
   * scan registration classes
   */
  virtual bool MatchScans(const ScanPose& scan_pose_ref,
                          const ScanPose& scan_pose_tgt,
                          Eigen::Matrix4d& T_LIDARREF_LIDARTGT) = 0;

  /**
   * @brief this function does 3 things:
   *
   *  1 - removes scans from the reference clouds list and unregistered clouds
   * list that are older than the lag duration and therefore cannot add
   * constraints to the graph anymore.
   *
   *  2 - removes scans from the reference clouds list if the list is larger
   * than the max number of neighbours param
   *
   *  3 - removes scans from the unregistered clouds list if the list is greater
   * than the max_unregistered_clouds_ param.
   */
  void CleanUpScanLists(const ros::Time& new_scan_time);

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

  // keep a list of reference clouds. These are scan poses which have already
  // been registered and are in the graph.
  std::list<ScanPose> reference_clouds_;

  // keep a list of unregistered clouds. These are scan poses which failed the
  // scan matching, often because they did not pass the motion threshold. We
  // want to keep some of these to see if they can eventually be registered to
  // another scan and included in the graph.
  std::list<ScanPose> unregistered_clouds_;

  // set a max amount of clouds to keep as unregistered. This can only be
  // editted here.
  uint32_t max_unregistered_clouds_{50};

  Params params_;

  const std::string source_{"MULTISCANREGISTRATION"};

  std::string current_scan_path_; // when output dir is set
  PointCloudCol coord_frame_;
};

class MultiScanLoamRegistration : public MultiScanRegistrationBase {
public:
  using Params = MultiScanRegistrationBase::Params;

  MultiScanLoamRegistration() = delete;

  MultiScanLoamRegistration(std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher,
                            const ScanRegistrationParamsBase& base_params,
                            int num_neighbors = 10, double lag_duration = 0,
                            bool disable_lidar_map = false);

private:
  bool MatchScans(const ScanPose& scan_pose_ref, const ScanPose& scan_pose_tgt,
                  Eigen::Matrix4d& T_LIDARREF_LIDARTGT) override;

  std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher_;
};

class MultiScanRegistration : public MultiScanRegistrationBase {
public:
  using Params = MultiScanRegistrationBase::Params;

  MultiScanRegistration() = delete;

  MultiScanRegistration(std::unique_ptr<Matcher<PointCloudPtr>> matcher,
                        const ScanRegistrationParamsBase& base_params,
                        int num_neighbors = 10, double lag_duration = 0,
                        bool disable_lidar_map = false);

private:
  bool MatchScans(const ScanPose& scan_pose_ref, const ScanPose& scan_pose_tgt,
                  Eigen::Matrix4d& T_LIDARREF_LIDARTGT) override;

  std::unique_ptr<Matcher<PointCloudPtr>> matcher_;
};

}} // namespace bs_models::scan_registration
