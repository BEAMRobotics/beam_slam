#pragma once

#include <beam_constraints/frame_to_frame/pose_3d_stamped_transaction.h>
#include <beam_common/scan_pose.h>

namespace beam_models {
namespace frame_to_frame {

template <typename ConstraintType, typename PriorType>
using TransactionBase =
    beam_constraints::frame_to_frame::FrameToFrameTransactionBase<
        ConstraintType, PriorType>;

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

 protected:
  Eigen::Matrix<double, 6, 6> covariance_;
  bool use_fixed_covariance_{false};
};

}  // namespace frame_to_frame
}  // namespace beam_models
