#include <bs_models/scan_registration/scan_to_map_registration.h>

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/transaction.h>

#include <beam_matching/Matchers.h>

#include <bs_common/utils.h>
#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>
#include <bs_models/scan_registration/registration_map.h>

namespace bs_models { namespace scan_registration {

using namespace beam_matching;
using namespace bs_common;

ScanToMapRegistrationBase::ScanToMapRegistrationBase(
    const ScanRegistrationParamsBase& base_params)
    : ScanRegistrationBase(base_params) {}

bs_constraints::relative_pose::Pose3DStampedTransaction
    ScanToMapRegistrationBase::RegisterNewScan(const ScanPose& new_scan) {
  bs_constraints::relative_pose::Pose3DStampedTransaction transaction(
      new_scan.Stamp());
  // add pose variables for new scan
  transaction.AddPoseVariables(new_scan.Position(), new_scan.Orientation(),
                               new_scan.Stamp());

  // if this is the first scan, we need to treat it differently
  if (scan_pose_prev_ == nullptr) {
    // if registration map is empty, then just add prior, add to map and return
    if (map_.Empty()) {
      Eigen::Matrix4d T_MAP_SCAN = new_scan.T_REFFRAME_LIDAR();
      AddScanToMap(new_scan, T_MAP_SCAN);
      if (base_params_.fix_first_scan) {
        transaction.AddPosePrior(new_scan.Position(), new_scan.Orientation(),
                                 pose_prior_noise_, "FIRSTSCANPRIOR");
      }
      scan_pose_prev_ = std::make_unique<ScanPose>(
          new_scan.Stamp(), new_scan.T_REFFRAME_BASELINK(),
          new_scan.T_BASELINK_LIDAR());
      return transaction;
    } else {
      // if map exists, then use the last scan in the map as the previous with a
      // prior
      ros::Time last_time = map_.GetLastLoamPoseStamp();
      if (last_time == ros::Time(0)) {
        last_time = map_.GetLastCloudPoseStamp();
      }
      Eigen::Matrix4d T_MAP_SCAN;
      map_.GetScanPose(last_time, T_MAP_SCAN);
      scan_pose_prev_ = std::make_unique<ScanPose>(
          last_time, T_MAP_SCAN * new_scan.T_LIDAR_BASELINK(),
          new_scan.T_BASELINK_LIDAR());
      if (base_params_.fix_first_scan) {
        transaction.AddPosePrior(scan_pose_prev_->Position(),
                                 scan_pose_prev_->Orientation(),
                                 pose_prior_noise_, "FIRSTSCANPRIOR");
      }
    }
  }

  Eigen::Matrix4d T_MAP_SCAN;
  if (!RegisterScanToMap(new_scan, T_MAP_SCAN)) {
    return bs_constraints::relative_pose::Pose3DStampedTransaction(
        new_scan.Stamp());
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
    std::string default_path = bs_common::GetBeamSlamConfigPath() +
                               "registration/scan_to_map_loam.json";
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

bool ScanToMapLoamRegistration::RegisterScanToMap(const ScanPose& scan_pose,
                                                  Eigen::Matrix4d& T_MAP_SCAN) {
  const Eigen::Matrix4d& T_MAPEST_SCAN = scan_pose.T_REFFRAME_LIDAR();
  const Eigen::Matrix4d& T_MAP_SCANPREV = scan_pose_prev_->T_REFFRAME_LIDAR();
  Eigen::Matrix4d T_SCANPREV_SCANNEW =
      beam::InvertTransform(T_MAP_SCANPREV) * T_MAPEST_SCAN;
  if (!PassedMotionThresholds(T_SCANPREV_SCANNEW)) { return false; }

  LoamPointCloudPtr scan_in_map_frame =
      std::make_shared<LoamPointCloud>(scan_pose.LoamCloud(), T_MAPEST_SCAN);
  // get combined loam cloud map
  LoamPointCloudPtr current_map =
      std::make_shared<LoamPointCloud>(map_.GetLoamCloudMap());
  matcher_->SetRef(current_map);
  matcher_->SetTarget(scan_in_map_frame);
  if (!matcher_->Match()) { return false; }
  if (!params_.debug_output_dir.empty()) {
    matcher_->SaveResults(params_.debug_output_dir,
                          std::to_string(scan_pose.Stamp().toSec()));
  }
  Eigen::Matrix4d T_MAPEST_MAP = matcher_->GetResult().matrix();

  if (!PassedRegThreshold(T_MAPEST_MAP)) {
    BEAM_WARN("Failed scan matcher transform threshold check for stamp {}.{}. "
              "Skipping measurement.",
              scan_pose.Stamp().sec, scan_pose.Stamp().nsec);
    std::cout << "T_MAPEST_MAP: \n" << T_MAPEST_MAP << "\n";
    return false;
  }
  T_MAP_SCAN = beam::InvertTransform(T_MAPEST_MAP) * T_MAPEST_SCAN;

  if (!use_fixed_covariance_) { covariance_ = matcher_->GetCovariance(); }
  return true;
}

void ScanToMapLoamRegistration::AddScanToMap(
    const ScanPose& scan_pose, const Eigen::Matrix4d& T_MAP_SCAN) {
  map_.AddPointCloud(scan_pose.LoamCloud(), scan_pose.Stamp(), T_MAP_SCAN);
  if (params_.store_full_cloud) {
    map_.AddPointCloud(scan_pose.Cloud(), scan_pose.Stamp(), T_MAP_SCAN);
  }
}

}} // namespace bs_models::scan_registration
