#include <beam_models/frame_to_frame/scan_registration/scan_registration_base.h>

#include <nlohmann/json.hpp>
#include <boost/filesystem.hpp>

namespace beam_models {
namespace frame_to_frame {

using namespace beam_matching;
using namespace beam_common;

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

const beam_common::LidarMap& ScanRegistrationBase::GetMap() const {
  return map_;
}

beam_common::LidarMap& ScanRegistrationBase::GetMapMutable() {
  return map_;
}

}  // namespace frame_to_frame
}  // namespace beam_models
