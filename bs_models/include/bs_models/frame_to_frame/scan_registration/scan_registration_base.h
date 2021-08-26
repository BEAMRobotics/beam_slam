#pragma once

#include <bs_constraints/frame_to_frame/pose_3d_stamped_transaction.h>
#include <bs_common/scan_pose.h>
#include <bs_common/lidar_map.h>

namespace bs_models {
namespace frame_to_frame {

template <typename ConstraintType, typename PriorType>
using TransactionBase =
    bs_constraints::frame_to_frame::FrameToFrameTransactionBase<ConstraintType,
                                                                PriorType>;

static std::string _tmp_string{""};

/**
 * @brief base class for scan registration parameters. These params will be used
 * in most scan regisatration implementations. This struct serves as a way to
 * eliminate code duplication. All scan registration implementations should have
 * a params class that inherits from this base class.
 */
struct ScanRegistrationParamsBase {
  /** all scan registration methods should check final registration result to
   * make sure it does not vary from initial guess by more than this amount.
   * This is an outlier removal methodology. We should be able to set this based
   * on how good we expect our initials estimates to be. Note that these are
   * "or" conditions, meaning if translation or rotation are violated, the
   * registration will be considered a failure. Empty transactions will be
   * returned if these fail. */
  double outlier_threshold_trans_m{0.3};
  double outlier_threshold_rot_deg{20};

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

  /** This will load the default params, and can be called by derived classes.
   */
  void LoadBaseFromJson(const std::string& config);
};

class ScanRegistrationBase {
 public:
  ScanRegistrationBase(const ScanRegistrationParamsBase& base_params);

  ~ScanRegistrationBase() = default;

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
  virtual bs_constraints::frame_to_frame::Pose3DStampedTransaction
  RegisterNewScan(const bs_common::ScanPose& new_scan) = 0;

  const bs_common::LidarMap& GetMap() const;

  bs_common::LidarMap& GetMapMutable();

 protected:
  bool PassedMotionThresholds(const Eigen::Matrix4d& T_CLOUD1_CLOUD2);

  bool PassedRegThreshold(const Eigen::Matrix4d& T_measured, std::string& summary = _tmp_string);

  ScanRegistrationParamsBase base_params_;
  Eigen::Matrix<double, 6, 6> covariance_;
  bool use_fixed_covariance_{false};
  bs_common::LidarMap& map_ = bs_common::LidarMap::GetInstance();

  /** This is the prior set on the first scan when fix_first_scan is enabled.
   * NOTE: this can only be set here, not in config. */
  double pose_prior_noise_{1e-9};

  /** Source to be added to the transaction */
  std::string base_source_{"SCANREGISTRATIONBASE"};
};

}  // namespace frame_to_frame
}  // namespace bs_models
