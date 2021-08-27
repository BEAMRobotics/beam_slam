#include <bs_models/scan_registration/scan_registration_base.h>

#include <nlohmann/json.hpp>
#include <beam_utils/filesystem.h>

namespace bs_models {
namespace scan_registration {

using namespace beam_matching;
using namespace bs_common;

void ScanRegistrationParamsBase::LoadBaseFromJson(const std::string& config) {
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

  nlohmann::json J;
  if(!beam::ReadJson(config, J)){
    BEAM_INFO("Using default config.");
    return;
  }
  
  outlier_threshold_trans_m = J["outlier_threshold_trans_m"];
  outlier_threshold_rot_deg = J["outlier_threshold_rot_deg"];
  min_motion_trans_m = J["min_motion_trans_m"];
  min_motion_rot_deg = J["min_motion_rot_deg"];
  max_motion_trans_m = J["max_motion_trans_m"];
  fix_first_scan = J["fix_first_scan"];
}

ScanRegistrationBase::ScanRegistrationBase(
    const ScanRegistrationParamsBase& base_params)
    : base_params_(base_params) {}

void ScanRegistrationBase::SetFixedCovariance(
    const Eigen::Matrix<double, 6, 6>& covariance) {
  covariance_ = covariance;
  use_fixed_covariance_ = true;
}

void ScanRegistrationBase::SetFixedCovariance(double covariance) {
  Eigen::VectorXd cov_vec(6);
  cov_vec << covariance, covariance, covariance, covariance, covariance,
      covariance;
  covariance_ = cov_vec.asDiagonal();
  use_fixed_covariance_ = true;
}

const RegistrationMap& ScanRegistrationBase::GetMap() const { return map_; }

RegistrationMap& ScanRegistrationBase::GetMapMutable() { return map_; }

bool ScanRegistrationBase::PassedRegThreshold(const Eigen::Matrix4d& T_measured,
                                              std::string& summary) {
  double t_error = T_measured.block(0, 3, 3, 1).norm();
  Eigen::Matrix3d R = T_measured.block(0, 0, 3, 3);
  double r_error_rad = std::abs(Eigen::AngleAxis<double>(R).angle());
  double r_error_deg = beam::Rad2Deg(r_error_rad);

  if (t_error > base_params_.outlier_threshold_trans_m) {
    summary = "Calculated translation is greater than threshold (" +
              std::to_string(t_error) + " > " +
              std::to_string(base_params_.outlier_threshold_trans_m) +
              ") - FAILED";
    return false;
  }

  if (r_error_deg > base_params_.outlier_threshold_rot_deg) {
    summary = "Calculated rotation is greater than threshold (" +
              std::to_string(r_error_deg) + " > " +
              std::to_string(base_params_.outlier_threshold_rot_deg) +
              ") - FAILED";
    return false;
  }

  summary = "PASSED";
  return true;
}

bool ScanRegistrationBase::PassedMotionThresholds(
    const Eigen::Matrix4d& T_CLOUD1_CLOUD2) {
  // check max translation
  double d_12 = T_CLOUD1_CLOUD2.block(0, 3, 3, 1).norm();
  if (d_12 > base_params_.max_motion_trans_m) {
    return false;
  }

  // check min translation
  if (d_12 >= base_params_.min_motion_trans_m) {
    return true;
  }

  // check rotation
  Eigen::Matrix3d R = T_CLOUD1_CLOUD2.block(0, 0, 3, 3);
  if (Eigen::AngleAxis<double>(R).angle() >= base_params_.min_motion_rot_deg) {
    return true;
  }
  return false;
}

}  // namespace scan_registration
}  // namespace bs_models
