#include <bs_models/scan_registration/scan_registration_base.h>

#include <filesystem>

#include <nlohmann/json.hpp>

#include <beam_matching/Matchers.h>
#include <beam_utils/angles.h>
#include <beam_utils/filesystem.h>

#include <bs_common/conversions.h>
#include <bs_models/scan_registration/multi_scan_registration.h>
#include <bs_models/scan_registration/scan_to_map_registration.h>

namespace bs_models { namespace scan_registration {

using namespace beam_matching;
using namespace bs_common;

std::unique_ptr<ScanRegistrationBase> ScanRegistrationBase::Create(
    const std::string& registration_config, const std::string& matcher_config,
    const std::string& save_path, double extrinsics_prior) {
  // get registration type
  if (!std::filesystem::exists(registration_config)) {
    BEAM_ERROR("invalid file path for matcher config, file path: {}",
               registration_config);
    throw std::invalid_argument{"invalid json"};
  }

  nlohmann::json J;
  std::ifstream file(registration_config);
  file >> J;
  if (!J.contains("registration_type")) {
    BEAM_ERROR("invalid registration config file, no registration_type field. "
               "Input file: {}",
               registration_config);
    throw std::invalid_argument{"invalid json"};
  }
  std::string registration_type = J["registration_type"];
  MatcherType matcher_type = beam_matching::GetTypeFromConfig(matcher_config);
  std::unique_ptr<scan_registration::ScanRegistrationBase> registration;

  // treat loam different
  if (matcher_type == beam_matching::MatcherType::LOAM) {
    std::string ceres_config = bs_common::GetAbsoluteConfigPathFromJson(
        matcher_config, "ceres_config");
    std::unique_ptr<LoamMatcher> matcher =
        std::make_unique<LoamMatcher>(LoamParams(matcher_config, ceres_config));

    if (registration_type == "SCANTOMAP") {
      ScanToMapLoamRegistration::Params params;
      params.LoadFromJson(registration_config);
      params.save_path = save_path;
      registration = std::make_unique<ScanToMapLoamRegistration>(
          std::move(matcher), params.GetBaseParams(), params.map_size,
          params.downsample_voxel_size);
      registration->SetExtrinsicsPrior(extrinsics_prior);
      return std::move(registration);
    } else if (registration_type == "MULTISCAN") {
      MultiScanRegistrationBase::Params params;
      params.LoadFromJson(registration_config);
      params.save_path = save_path;
      registration = std::make_unique<MultiScanLoamRegistration>(
          std::move(matcher), params.GetBaseParams(), params.num_neighbors,
          params.lag_duration, params.disable_lidar_map);
      registration->SetExtrinsicsPrior(extrinsics_prior);
      return std::move(registration);
    } else {
      BEAM_ERROR("registration type not yet implemented");
      throw std::runtime_error{"function not implemented"};
    }
  }

  // non-loam, only multi scan is implemented so far
  std::unique_ptr<Matcher<PointCloudPtr>> matcher;
  MultiScanRegistrationBase::Params params;
  params.LoadFromJson(registration_config);
  params.save_path = save_path;
  if (registration_type == "MULTISCAN") {
    if (matcher_type == beam_matching::MatcherType::ICP) {
      matcher =
          std::make_unique<IcpMatcher>(IcpMatcher::Params(matcher_config));
      registration = std::make_unique<MultiScanRegistration>(
          std::move(matcher), params.GetBaseParams(), params.num_neighbors,
          params.lag_duration, params.disable_lidar_map);
    } else if (matcher_type == beam_matching::MatcherType::GICP) {
      matcher =
          std::make_unique<GicpMatcher>(GicpMatcher::Params(matcher_config));
      registration = std::make_unique<MultiScanRegistration>(
          std::move(matcher), params.GetBaseParams(), params.num_neighbors,
          params.lag_duration, params.disable_lidar_map);
    } else if (matcher_type == beam_matching::MatcherType::NDT) {
      matcher =
          std::make_unique<NdtMatcher>(NdtMatcher::Params(matcher_config));
      registration = std::make_unique<MultiScanRegistration>(
          std::move(matcher), params.GetBaseParams(), params.num_neighbors,
          params.lag_duration, params.disable_lidar_map);
    } else {
      ROS_ERROR(
          "Invalid global matcher type. Not creating scan registration class");
      throw std::invalid_argument{"invalid json"};
    }
  } else {
    BEAM_ERROR("registration type not yet implemented");
    throw std::runtime_error{"function not implemented"};
  }

  registration->SetExtrinsicsPrior(extrinsics_prior);
  return std::move(registration);
}

void ScanRegistrationParamsBase::Print(std::ostream& stream) const {
  stream << "ScanRegistrationParamsBase: \n";
  stream << "min_motion_trans_m: " << min_motion_trans_m << "\n";
  stream << "min_motion_rot_deg: " << min_motion_rot_deg << "\n";
  stream << "max_motion_trans_m: " << max_motion_trans_m << "\n";
  stream << "fix_first_scan: " << fix_first_scan << "\n";
  stream << "save_path: " << save_path << "\n";
}

void ScanRegistrationParamsBase::LoadBaseFromJson(const std::string& config) {
  // check file exists
  if (config.empty()) {
    return;
  } else if (!std::filesystem::exists(config)) {
    BEAM_WARN(
        "Invalid scan registration config path, file does not exist, using "
        "default. Input: {}",
        config);
    return;
  }

  nlohmann::json J;
  if (!beam::ReadJson(config, J)) {
    BEAM_INFO("Using default config.");
    return;
  }

  beam::ValidateJsonKeysOrThrow({"min_motion_trans_m", "min_motion_rot_deg",
                                 "max_motion_trans_m", "fix_first_scan"},
                                J);

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

const RegistrationMap& ScanRegistrationBase::GetMap() const {
  return map_;
}

RegistrationMap& ScanRegistrationBase::GetMapMutable() {
  return map_;
}

bool ScanRegistrationBase::PassedMotionThresholds(
    const Eigen::Matrix4d& T_CLOUD1_CLOUD2) {
  // check max translation
  double d_12 = T_CLOUD1_CLOUD2.block(0, 3, 3, 1).norm();
  if (base_params_.max_motion_trans_m > 0 &&
      d_12 > base_params_.max_motion_trans_m) {
    return false;
  }

  // check min translation
  bool passed_trans{true};
  if (base_params_.min_motion_trans_m > 0 &&
      d_12 < base_params_.min_motion_trans_m) {
    passed_trans = false;
  }

  // check min rotation
  bool passed_rot{true};
  Eigen::Matrix3d R = T_CLOUD1_CLOUD2.block(0, 0, 3, 3);
  double angle_rad = Eigen::AngleAxis<double>(R).angle();
  if (base_params_.min_motion_rot_deg > 0 &&
      beam::Rad2Deg(angle_rad) < base_params_.min_motion_rot_deg) {
    passed_rot = false;
  }

  if (base_params_.min_motion_rot_deg == 0) {
    return passed_trans;
  } else if (base_params_.min_motion_trans_m == 0) {
    return passed_rot;
  }

  return (passed_trans || passed_rot);
}

void ScanRegistrationBase::SetInformationWeight(double w) {
  covariance_weight_ = 1 / (w * w);
}

}} // namespace bs_models::scan_registration
