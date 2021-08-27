#include <bs_models/frame_to_frame/scan_registration/scan_to_map_registration.h>

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/transaction.h>

#include <beam_matching/Matchers.h>

#include <bs_common/utils.h>
#include <bs_common/lidar_map.h>
#include <bs_constraints/frame_to_frame/pose_3d_stamped_transaction.h>

namespace bs_models {
namespace frame_to_frame {

using namespace beam_matching;
using namespace bs_common;

ScanToMapRegistrationBase::ScanToMapRegistrationBase(
    const ScanRegistrationParamsBase& base_params)
    : ScanRegistrationBase(base_params) {}

bs_constraints::frame_to_frame::Pose3DStampedTransaction
ScanToMapRegistrationBase::RegisterNewScan(const ScanPose& new_scan) {
  bs_constraints::frame_to_frame::Pose3DStampedTransaction transaction(
      new_scan.Stamp());

  // add pose variables for new scan
  transaction.AddPoseVariables(new_scan.Position(), new_scan.Orientation(),
                               new_scan.Stamp());

  // if map is empty, then add to the map then return
  Eigen::Matrix4d T_MAP_SCAN;
  if (IsMapEmpty()) {
    T_MAP_SCAN = new_scan.T_REFFRAME_LIDAR();
    AddScanToMap(new_scan, T_MAP_SCAN);
    if (base_params_.fix_first_scan) {
      // build covariance
      fuse_core::Matrix6d prior_covariance;
      prior_covariance.setIdentity();
      prior_covariance = prior_covariance * pose_prior_noise_;

      // add prior
      transaction.AddPosePrior(new_scan.Position(), new_scan.Orientation(),
                               prior_covariance, "FIRSTSCANPRIOR");
    }

    scan_pose_prev_ = std::make_unique<ScanPose>(
        new_scan.Stamp(), new_scan.T_REFFRAME_BASELINK(),
        new_scan.T_BASELINK_LIDAR());

    return transaction;
  }

  if (!RegisterScanToMap(new_scan, T_MAP_SCAN)) {
    return transaction;
  }

  /**
   * We need to convert the relative poses measurements from lidar (or cloud)
   * frames to baselink frames:
   *
   * T_BASELINKPREV_BASELINKNEW = inv(T_MAP_BASELINKPREV) * T_MAP_BASELINKNEW
   */
  Eigen::Matrix4d T_BASELINKPREV_MAP =
      beam::InvertTransform(scan_pose_prev_->T_REFFRAME_BASELINK());
  Eigen::Matrix4d T_MAP_BASELINKNEW = T_MAP_SCAN * new_scan.T_LIDAR_BASELINK();
  Eigen::Matrix4d T_BASELINKPREV_BASELINKNEW =
      T_BASELINKPREV_MAP * T_MAP_BASELINKNEW;
  fuse_variables::Position3DStamped position_relative;
  fuse_variables::Orientation3DStamped orientation_relative;
  bs_common::EigenTransformToFusePose(T_BASELINKPREV_BASELINKNEW,
                                      position_relative, orientation_relative);

  // add measurement to transaction
  transaction.AddPoseConstraint(
      scan_pose_prev_->Position(), new_scan.Position(),
      scan_pose_prev_->Orientation(), new_scan.Orientation(), position_relative,
      orientation_relative, covariance_, source_);

  // add new registered scan and then trim the map
  AddScanToMap(new_scan, T_MAP_SCAN);

  // copy over just pose information
  scan_pose_prev_ = std::make_unique<ScanPose>(
      new_scan.Stamp(), T_MAP_SCAN * new_scan.T_LIDAR_BASELINK(),
      new_scan.T_BASELINK_LIDAR());

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
  } else if (config == "DEFAULT_PATH") {
    std::string default_path = __FILE__;
    size_t start_iter = default_path.find("bs_models");
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
    BEAM_INFO("Reading scan to map loam registration default config file: {}",
              default_path);
    read_file = default_path;
  } else if (!boost::filesystem::exists(config)) {
    BEAM_WARN(
        "Invalid scan registration config path, file does not exist, using "
        "default. Input: {}",
        config);
    return;
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

ScanRegistrationParamsBase ScanToMapLoamRegistration::Params::GetBaseParams() {
  ScanRegistrationParamsBase base_params{
      .outlier_threshold_trans_m = outlier_threshold_trans_m,
      .outlier_threshold_rot_deg = outlier_threshold_rot_deg,
      .min_motion_trans_m = min_motion_trans_m,
      .min_motion_rot_deg = min_motion_rot_deg,
      .max_motion_trans_m = max_motion_trans_m,
      .fix_first_scan = fix_first_scan};
  return base_params;
}

ScanToMapLoamRegistration::ScanToMapLoamRegistration(
    std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher,
    const ScanRegistrationParamsBase& base_params, int map_size,
    bool store_full_cloud)
    : ScanToMapRegistrationBase(base_params), matcher_(std::move(matcher)) {
  params_.map_size = map_size;
  params_.store_full_cloud = store_full_cloud;
  map_.SetParams(params_.map_size);
}

bool ScanToMapLoamRegistration::IsMapEmpty() {
  return map_.NumLoamClouds() == 0;
}

bool ScanToMapLoamRegistration::RegisterScanToMap(const ScanPose& scan_pose,
                                                  Eigen::Matrix4d& T_MAP_SCAN) {
  Eigen::Matrix4d T_MAPEST_SCAN = scan_pose.T_REFFRAME_LIDAR();
  Eigen::Matrix4d T_MAP_SCANPREV = scan_pose_prev_->T_REFFRAME_LIDAR();

  Eigen::Matrix4d T_SCANPREV_SCANNEW =
      beam::InvertTransform(T_MAP_SCANPREV) * T_MAPEST_SCAN;

  if (!PassedMotionThresholds(T_SCANPREV_SCANNEW)) {
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

  std::string summary;
  if (!PassedRegThreshold(T_MAPEST_MAP, summary)) {
    BEAM_WARN(
        "Failed scan matcher transform threshold check for stamp {}.{}. "
        "Skipping measurement. Reason: {}",
        scan_pose.Stamp().sec, scan_pose.Stamp().nsec, summary);
    return false;
  }

  T_MAP_SCAN = beam::InvertTransform(T_MAPEST_MAP) * T_MAPEST_SCAN;

  return true;
}

void ScanToMapLoamRegistration::AddScanToMap(
    const ScanPose& scan_pose, const Eigen::Matrix4d& T_MAP_SCAN) {
  map_.AddPointCloud(scan_pose.LoamCloud(), scan_pose.Stamp(), T_MAP_SCAN);
  if (params_.store_full_cloud) {
    map_.AddPointCloud(scan_pose.Cloud(), scan_pose.Stamp(), T_MAP_SCAN);
  }
}

}  // namespace frame_to_frame
}  // namespace bs_models
