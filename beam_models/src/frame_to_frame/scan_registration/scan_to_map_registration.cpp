#include <beam_models/frame_to_frame/scan_registration/scan_to_map_registration.h>

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/transaction.h>

#include <beam_matching/Matchers.h>

#include <beam_common/utils.h>
#include <beam_common/lidar_map.h>
#include <beam_constraints/frame_to_frame/pose_3d_stamped_transaction.h>

namespace beam_models {
namespace frame_to_frame {

using namespace beam_matching;
using namespace beam_common;

ScanToMapRegistrationBase::ScanToMapRegistrationBase(bool fix_first_scan)
    : fix_first_scan_(fix_first_scan) {}

beam_constraints::frame_to_frame::Pose3DStampedTransaction
ScanToMapRegistrationBase::RegisterNewScan(const ScanPose& new_scan) {
  beam_constraints::frame_to_frame::Pose3DStampedTransaction transaction(
      new_scan.Stamp());

  // add pose variables for new scan
  transaction.AddPoseVariables(new_scan.Position(), new_scan.Orientation(),
                               new_scan.Stamp());

  // if map is empty, then add to the map then return
  Eigen::Matrix4d T_MAP_SCAN;
  if (IsMapEmpty()) {
    T_MAP_SCAN = new_scan.T_REFFRAME_CLOUD();
    AddScanToMap(new_scan, T_MAP_SCAN);
    if (fix_first_scan_) {
      // build covariance
      fuse_core::Matrix6d prior_covariance;
      prior_covariance.setIdentity();
      prior_covariance = prior_covariance * pose_prior_noise_;

      // add prior
      transaction.AddPosePrior(new_scan.Position(), new_scan.Orientation(),
                               prior_covariance, "FIRSTSCANPRIOR");
    }
    T_MAP_SCANPREV_ = T_MAP_SCAN;
    scan_prev_position_ = new_scan.Position();
    scan_prev_orientation_ = new_scan.Orientation();
    return transaction;
  }

  if (!RegisterScanToMap(new_scan, T_MAP_SCAN)) {
    return transaction;
  }

  // add measurement to transaction
  fuse_variables::Position3DStamped position_relative;
  fuse_variables::Orientation3DStamped orientation_relative;
  Eigen::Matrix4d T_SCANPREV_SCANNEW =
      beam::InvertTransform(T_MAP_SCANPREV_) * T_MAP_SCAN;
  beam_common::EigenTransformToFusePose(T_SCANPREV_SCANNEW, position_relative,
                                        orientation_relative);
  transaction.AddPoseConstraint(scan_prev_position_, new_scan.Position(),
                                scan_prev_orientation_, new_scan.Orientation(),
                                position_relative, orientation_relative,
                                covariance_, source_);

  // add new registered scan and then trim the map
  AddScanToMap(new_scan, T_MAP_SCAN);

  T_MAP_SCANPREV_ = T_MAP_SCAN;
  scan_prev_position_ = new_scan.Position();
  scan_prev_orientation_ = new_scan.Orientation();
  return transaction;
}

ScanToMapLoamRegistration::Params::Params(
    const ScanRegistrationParamsBase& base_params, int _map_size,
    bool _store_full_cloud)
    : ScanRegistrationParamsBase(base_params),
      map_size(_map_size),
      store_full_cloud(_store_full_cloud) {}

void ScanToMapLoamRegistration::Params::LoadFromJson(
    const std::string& config) {
  std::string read_file = config;
  if (config.empty()) {
    return;
  } else if (!boost::filesystem::exists(config)) {
    BEAM_WARN(
        "Invalid scan registration config path, file does not exist, using "
        "default. Input: {}",
        config);
    return;
  } else if (config == "DEFAULT_PATH") {
    std::string default_path = __FILE__;
    size_t start_iter = default_path.find("beam_models");
    size_t end_iter = default_path.size() - start_iter;
    default_path.erase(start_iter, end_iter);
    default_path +=
        "beam_slam_launch/config/registration_config/scan_to_map_loam.json";
    if (!boost::filesystem::exists(default_path)) {
      BEAM_WARN(
          "Could not find default multi scan registration config at: {}. Using "
          "default params.",
          default_path);
      return;
    }
    read_file = default_path;
  }

  // load default params
  LoadBaseFromJson(read_file);

  // load other params specific to this class
  nlohmann::json J;
  std::ifstream file(read_file);
  file >> J;

  map_size = J["map_size"];
  store_full_cloud = J["store_full_cloud"];
}

ScanToMapLoamRegistration::ScanToMapLoamRegistration(
    std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher, const Params& params)
    : ScanToMapRegistrationBase(params.fix_first_scan),
      matcher_(std::move(matcher)),
      params_(params) {
  map_.SetParams(params_.map_size);
}

bool ScanToMapLoamRegistration::IsMapEmpty() {
  return map_.NumLoamClouds() == 0;
}

bool ScanToMapLoamRegistration::RegisterScanToMap(const ScanPose& scan_pose,
                                                  Eigen::Matrix4d& T_MAP_SCAN) {
  const Eigen::Matrix4d& T_MAPEST_SCAN = scan_pose.T_REFFRAME_CLOUD();
  Eigen::Matrix4d T_SCANPREV_SCANNEW =
      beam::InvertTransform(T_MAP_SCANPREV_) * T_MAPEST_SCAN;
  if (!PassedMinMotion(T_SCANPREV_SCANNEW)) {
    return false;
  }

  LoamPointCloudPtr scan_in_map_frame =
      std::make_shared<LoamPointCloud>(scan_pose.LoamCloud());
  scan_in_map_frame->TransformPointCloud(T_MAPEST_SCAN);

  // get combined loamcloud map
  LoamPointCloudPtr current_map =
      std::make_shared<LoamPointCloud>(map_.GetLoamCloudMap());

  matcher_->SetRef(current_map);
  matcher_->SetTarget(scan_in_map_frame);

  if (!matcher_->Match()) {
    return false;
  }

  Eigen::Matrix4d T_MAPEST_MAP = matcher_->GetResult().matrix();

  if (!PassedRegThreshold(T_MAPEST_MAP)) {
    return false;
  }

  T_MAP_SCAN = beam::InvertTransform(T_MAPEST_MAP) * T_MAPEST_SCAN;

  return true;
}

void ScanToMapLoamRegistration::AddScanToMap(
    const ScanPose& scan_pose, const Eigen::Matrix4d& T_MAP_SCAN) {
  map_.AddPointCloud(scan_pose.LoamCloud(), scan_pose.Stamp(), T_MAP_SCAN);
  if(params_.store_full_cloud){
    map_.AddPointCloud(scan_pose.Cloud(), scan_pose.Stamp(), T_MAP_SCAN);
  }
  
}

bool ScanToMapLoamRegistration::PassedRegThreshold(
    const Eigen::Matrix4d& T_measured) {
  double t_error = T_measured.block(0, 3, 3, 1).norm();
  Eigen::Matrix3d R = T_measured.block(0, 0, 3, 3);
  double r_error = std::abs(Eigen::AngleAxis<double>(R).angle());

  if (t_error > params_.outlier_threshold_t ||
      r_error > params_.outlier_threshold_r) {
    return false;
  }
  return true;
}

bool ScanToMapLoamRegistration::PassedMinMotion(
    const Eigen::Matrix4d& T_CLOUD1_CLOUD2) {
  // check translation
  if (T_CLOUD1_CLOUD2.block(0, 3, 3, 1).norm() >= params_.min_motion_trans_m) {
    return true;
  }

  // check rotation
  Eigen::Matrix3d R = T_CLOUD1_CLOUD2.block(0, 0, 3, 3);
  if (Eigen::AngleAxis<double>(R).angle() >= params_.min_motion_rot_rad) {
    return true;
  }
  return false;
}

}  // namespace frame_to_frame
}  // namespace beam_models
