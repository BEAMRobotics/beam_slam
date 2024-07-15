#include <bs_models/scan_registration/scan_to_map_registration.h>

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/transaction.h>

#include <beam_matching/Matchers.h>

#include <bs_common/conversions.h>
#include <bs_common/utils.h>
#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>
#include <bs_models/scan_registration/registration_map.h>

namespace bs_models { namespace scan_registration {

using namespace beam_matching;
using namespace bs_common;

ScanToMapRegistrationBase::ScanToMapRegistrationBase(
    const ScanRegistrationParamsBase& base_params)
    : ScanRegistrationBase(base_params) {}

bs_constraints::Pose3DStampedTransaction
    ScanToMapRegistrationBase::RegisterNewScan(const ScanPose& new_scan) {
  bs_constraints::Pose3DStampedTransaction transaction(new_scan.Stamp());
  // add pose variables for new scan
  transaction.AddPoseVariables(new_scan.Position(), new_scan.Orientation(),
                               new_scan.Stamp());

  // if this is the first scan, we need to treat it differently
  if (scan_pose_prev_ == nullptr) {
    transaction.AddExtrinsicVariablesForFrame(extrinsics_.GetLidarFrameId(),
                                              extrinsics_prior_);

    // if registration map is empty, then just add prior, add to map and return
    if (map_.Empty()) {
      Eigen::Matrix4d T_MAP_SCAN = new_scan.T_REFFRAME_LIDAR();
      AddScanToMap(new_scan, T_MAP_SCAN);
      if (base_params_.fix_first_scan) {
        transaction.AddPosePrior(new_scan.Position(), new_scan.Orientation(),
                                 pose_prior_noise_,
                                 "LidarOdometry::ScanToMapRegistration");
      }
      scan_pose_prev_ = std::make_unique<ScanPose>(
          new_scan.Stamp(), new_scan.T_REFFRAME_BASELINK(),
          new_scan.T_BASELINK_LIDAR());
      return transaction;
    } else {
      // if map exists, then use the last scan in the map as the previous with a
      // prior
      ros::Time last_time = map_.GetLastCloudPoseStamp();
      Eigen::Matrix4d T_MAP_SCAN;
      map_.GetScanPose(last_time, T_MAP_SCAN);
      scan_pose_prev_ = std::make_unique<ScanPose>(
          last_time, T_MAP_SCAN * new_scan.T_LIDAR_BASELINK(),
          new_scan.T_BASELINK_LIDAR());
      if (base_params_.fix_first_scan) {
        transaction.AddPosePrior(
            scan_pose_prev_->Position(), scan_pose_prev_->Orientation(),
            pose_prior_noise_, "LidarOdometry::ScanToMapRegistration");
      }
    }
  }

  Eigen::Matrix4d T_MAP_SCAN;
  if (!RegisterScanToMap(new_scan, T_MAP_SCAN)) {
    return bs_constraints::Pose3DStampedTransaction(new_scan.Stamp());
  }

  Eigen::Matrix4d T_ScanPrev_Map =
      beam::InvertTransform(scan_pose_prev_->T_REFFRAME_LIDAR());
  Eigen::Matrix4d T_LidarPrev_LidarNew = T_ScanPrev_Map * T_MAP_SCAN;

  // add measurement to transaction
  transaction.AddPoseConstraint(
      scan_pose_prev_->Position(), new_scan.Position(),
      scan_pose_prev_->Orientation(), new_scan.Orientation(),
      bs_common::TransformMatrixToVectorWithQuaternion(T_LidarPrev_LidarNew),
      covariance_weight_ * covariance_, source_, extrinsics_.GetLidarFrameId());

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
    double _downsample_voxel_size)
    : ScanRegistrationParamsBase(base_params),
      map_size(_map_size),
      downsample_voxel_size(_downsample_voxel_size) {}

void ScanToMapLoamRegistration::Params::LoadFromJson(
    const std::string& config) {
  std::string read_file = config;
  if (config.empty()) {
    BEAM_INFO("Config file empty, using default parameters");
    return;
  } else if (config == "DEFAULT_PATH") {
    std::string default_path =
        bs_common::GetBeamSlamConfigPath() + "registration/scan_to_map.json";
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
  if (!beam::ReadJson(read_file, J)) {
    BEAM_ERROR("Unable to json");
    throw std::runtime_error{"Unable to read config"};
  }
  beam::ValidateJsonKeysOrThrow({"downsample_voxel_size", "map_size"}, J);

  map_size = J["map_size"];
  downsample_voxel_size = J["downsample_voxel_size"];
}

void ScanToMapLoamRegistration::Params::Print(std::ostream& stream) const {
  GetBaseParams().Print(stream);
  stream << "ScanToMapLoamRegistration::Params: \n";
  stream << "map_size: " << map_size << "\n";
  stream << "downsample_voxel_size: " << downsample_voxel_size << "\n";
}

ScanRegistrationParamsBase
    ScanToMapLoamRegistration::Params::GetBaseParams() const {
  ScanRegistrationParamsBase base_params{
      .min_motion_trans_m = min_motion_trans_m,
      .min_motion_rot_deg = min_motion_rot_deg,
      .max_motion_trans_m = max_motion_trans_m,
      .fix_first_scan = fix_first_scan,
      .save_path = save_path};
  return base_params;
}

ScanToMapLoamRegistration::ScanToMapLoamRegistration(
    std::unique_ptr<LoamMatcher> matcher,
    const ScanRegistrationParamsBase& base_params, int map_size,
    double downsample_voxel_size)
    : ScanToMapRegistrationBase(base_params),
      matcher_(std::move(matcher)),
      params_(base_params, map_size, downsample_voxel_size) {
  map_.SetMapSize(params_.map_size);
  map_.SetVoxelDownsampleSize(params_.downsample_voxel_size);
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
  if (!params_.save_path.empty()) {
    matcher_->SaveResults(params_.save_path,
                          std::to_string(scan_pose.Stamp().toSec()));
  }
  Eigen::Matrix4d T_MAPEST_MAP = matcher_->GetResult().matrix();

  if (!use_fixed_covariance_) { covariance_ = matcher_->GetCovariance(); }

  if (registration_validation_.Validate(T_MAPEST_MAP, covariance_)) {
    T_MAP_SCAN = beam::InvertTransform(T_MAPEST_MAP) * T_MAPEST_SCAN;
    return true;
  }
  return false;
}

void ScanToMapLoamRegistration::AddScanToMap(
    const ScanPose& scan_pose, const Eigen::Matrix4d& T_MAP_SCAN) {
  map_.AddPointCloud(scan_pose.Cloud(), scan_pose.LoamCloud(),
                     scan_pose.Stamp(), T_MAP_SCAN);
}

}} // namespace bs_models::scan_registration
