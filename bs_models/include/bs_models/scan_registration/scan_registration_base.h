#pragma once

#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>
#include <bs_models/lidar/scan_pose.h>
#include <bs_models/scan_registration/registration_map.h>
#include <bs_models/scan_registration/registration_validation.h>

namespace bs_models { namespace scan_registration {

/**
 * @brief base class for scan registration parameters. These params will be used
 * in most scan regisatration implementations. This struct serves as a way to
 * eliminate code duplication. All scan registration implementations should have
 * a params class that inherits from this base class.
 */
struct ScanRegistrationParamsBase {
  /** In some cases, we may not want to register scans that are too close
   * together. Setting these values to something >0 will enforce this
   * constraint. Empty transactions will be returned if they violate this
   * constraint. Not this is an or, not and. Only needs to pass one. */
  double min_motion_trans_m{0};
  double min_motion_rot_deg{0};

  /** We want to make sure we don't try to register scans that have very little
   * overlap, this param helps reduce that chance. Set this to avoid registering
   * a new scan if the estimated distance from this scan to the last scan is
   * greater than this threshold. Set to zero to avoid this constraint*/
  double max_motion_trans_m{10};

  /** If set to true, the scan registration method will set an almost perfect
   * prior to the pose of the first scan. */
  bool fix_first_scan{false};

  /** If not empty, each method can save registration output to this path */
  std::string save_path;

  /** This will load the default params, and can be called by derived classes.
   */
  void LoadBaseFromJson(const std::string& config);

  void Print(std::ostream& stream = std::cout) const;
};

class ScanRegistrationBase {
public:
  ScanRegistrationBase(const ScanRegistrationParamsBase& base_params);

  ~ScanRegistrationBase() = default;

  /**
   * @brief Factory method to create a scan registration object at runtime
   */
  static std::unique_ptr<ScanRegistrationBase>
      Create(const std::string& registration_config,
             const std::string& matcher_config,
             const std::string& save_path = "");

  void SetFixedCovariance(const Eigen::Matrix<double, 6, 6>& covariance);

  void SetFixedCovariance(double covariance);

  /**
   * @brief pure virtual function that each derived class must implement. The
   * function must generate a frame to frame transaction of type
   * Pose3DStampedTransaction, where the poses MUST BE IN BASELINK FRAME.
   * Generally, the scan registration is done in the lidar frames (to avoid
   * uneccesary data conversions) and then the relative poses are transformed to
   * relative baselink poses given the extrinsics
   */
  virtual bs_constraints::Pose3DStampedTransaction
      RegisterNewScan(const ScanPose& new_scan) = 0;

  const RegistrationMap& GetMap() const;

  RegistrationMap& GetMapMutable();

  void SetInformationWeight(double w);

protected:
  bool PassedMotionThresholds(const Eigen::Matrix4d& T_CLOUD1_CLOUD2);

  ScanRegistrationParamsBase base_params_;
  Eigen::Matrix<double, 6, 6> covariance_;
  bool use_fixed_covariance_{false};
  RegistrationMap& map_ = RegistrationMap::GetInstance();
  RegistrationValidation registration_validation_;
  double covariance_weight_{1.0};

  /** This is the prior set on the first scan when fix_first_scan is enabled.
   * NOTE: this can only be set here, not in config. */
  double pose_prior_noise_{1e-9};

  /** Source to be added to the transaction */
  std::string base_source_{"LidarOdometry::RegistrationBase"};
};

}} // namespace bs_models::scan_registration
