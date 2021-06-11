#pragma once

#include <nlohmann/json.hpp>
#include <boost/filesystem.hpp>

#include <beam_constraints/frame_to_frame/pose_3d_stamped_transaction.h>
#include <beam_common/scan_pose.h>
#include <beam_common/lidar_map.h>

namespace beam_models {
namespace frame_to_frame {

template <typename ConstraintType, typename PriorType>
using TransactionBase =
    beam_constraints::frame_to_frame::FrameToFrameTransactionBase<
        ConstraintType, PriorType>;

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
  double outlier_threshold_t{0.3};
  double outlier_threshold_r{20};

  /** In some cases, we may not want to register scans that are too close
   * together. Setting these values to something >0 will enforce this
   * constraint. Empty transactions will be returned if they violate this
   * constraint.*/
  double min_motion_trans_m{0};
  double min_motion_rot_rad{0};

  /** Source to be added to the transaction */
  std::string source{"SCANREGISTRATIONBASE"};

  /** If set to true, the scan registration method will set an almost perfect
   * prior to the pose of the first scan. */
  bool fix_first_scan{false};

  /** This will load the default params, and can be called by derived classes.
   */
  void LoadBaseFromJson(const std::string& config) {
    // check file exists
    if (config.empty()) {
      return;
    } else if (!boost::filesystem::exists(config)) {
      BEAM_WARN(
          "Invalid scan registration config path, file does not exist, using "
          "default. Input: {}",
          config);
      return;
    }

    BEAM_INFO("Loading scan registration config file: {}", config);

    nlohmann::json J;
    std::ifstream file(config);
    file >> J;

    outlier_threshold_t = J["outlier_threshold_t"];
    outlier_threshold_r = J["outlier_threshold_r"];
    min_motion_trans_m = J["min_motion_trans_m"];
    min_motion_rot_rad = J["min_motion_rot_rad"];
    source = J["source"];
    fix_first_scan = J["fix_first_scan"];
  }
};

class ScanRegistrationBase {
 public:
  ScanRegistrationBase() = default;

  ~ScanRegistrationBase() = default;

  inline void SetFixedCovariance(
      const Eigen::Matrix<double, 6, 6>& covariance) {
    covariance_ = covariance;
    use_fixed_covariance_ = true;
  }

  inline void SetFixedCovariance(double covariance) {
    Eigen::VectorXd cov_vec(6);
    cov_vec << covariance, covariance, covariance, covariance, covariance,
        covariance;
    covariance_ = cov_vec.asDiagonal();
    use_fixed_covariance_ = true;
  }

  virtual beam_constraints::frame_to_frame::Pose3DStampedTransaction
  RegisterNewScan(const beam_common::ScanPose& new_scan) = 0;

  inline const beam_common::LidarMap& GetMap() const {
    return map_;
  }

 protected:
  Eigen::Matrix<double, 6, 6> covariance_;
  bool use_fixed_covariance_{false};
  beam_common::LidarMap& map_ = beam_common::LidarMap::GetInstance();
};

}  // namespace frame_to_frame
}  // namespace beam_models
