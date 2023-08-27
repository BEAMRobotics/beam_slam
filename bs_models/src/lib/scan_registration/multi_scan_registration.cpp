#include <bs_models/scan_registration/multi_scan_registration.h>

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/transaction.h>

#include <beam_matching/Matchers.h>

#include <bs_common/conversions.h>
#include <bs_common/utils.h>
#include <bs_constraints/relative_pose/pose_3d_stamped_transaction.h>

namespace bs_models { namespace scan_registration {

using namespace beam_matching;
using namespace bs_common;

MultiScanRegistrationBase::Params::Params(
    const ScanRegistrationParamsBase& base_params, int _num_neighbors,
    double _lag_duration, bool _disable_lidar_map)
    : ScanRegistrationParamsBase(base_params),
      num_neighbors(_num_neighbors),
      lag_duration(_lag_duration),
      disable_lidar_map(_disable_lidar_map) {}

void MultiScanRegistrationBase::Params::LoadFromJson(
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
    std::string default_path = bs_common::GetBeamSlamConfigPath() +
                               "registration_config/multi_scan.json";
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

  num_neighbors = J["num_neighbors"];
  disable_lidar_map = J["disable_lidar_map"];
}

ScanRegistrationParamsBase MultiScanRegistrationBase::Params::GetBaseParams() {
  ScanRegistrationParamsBase base_params{
      .min_motion_trans_m = min_motion_trans_m,
      .min_motion_rot_deg = min_motion_rot_deg,
      .max_motion_trans_m = max_motion_trans_m,
      .fix_first_scan = fix_first_scan};
  return base_params;
}

MultiScanRegistrationBase::MultiScanRegistrationBase(
    const ScanRegistrationParamsBase& base_params, int num_neighbors,
    double lag_duration, bool disable_lidar_map)
    : ScanRegistrationBase(base_params),
      params_(base_params, num_neighbors, lag_duration, disable_lidar_map) {
  params_.num_neighbors = num_neighbors;
  params_.lag_duration = lag_duration;
  params_.disable_lidar_map = disable_lidar_map;
}

bs_constraints::Pose3DStampedTransaction
    MultiScanRegistrationBase::RegisterNewScan(const ScanPose& new_scan) {
  // create empty transaction
  bs_constraints::Pose3DStampedTransaction transaction(new_scan.Stamp());

  // if first scan, add to list then exit
  if (reference_clouds_.empty()) {
    AddFirstScan(new_scan, transaction);
    return transaction;
  }

  CleanUpScanLists(new_scan.Stamp());

  // first, let's go through the unregistered scans and try to register them to
  // a scan in the reference scans
  auto unreg_iter = unregistered_clouds_.begin();
  while (unreg_iter != unregistered_clouds_.end()) {
    int num_measurements = RegisterScanToReferences(*unreg_iter, transaction);

    if (num_measurements > 0) {
      ROS_DEBUG("Adding %d measurements to unregistered scan with stamp %d.%d.",
                num_measurements, new_scan.Stamp().sec, new_scan.Stamp().nsec);
      InsertCloudInReferences(*unreg_iter);

      // add pose variables for this scan
      transaction.AddPoseVariables(unreg_iter->Position(),
                                   unreg_iter->Orientation(),
                                   unreg_iter->Stamp());

      unregistered_clouds_.erase(unreg_iter++);
    } else {
      ++unreg_iter;
    }
  }

  // now, let's register the new scan to the reference scans
  int num_new_measurements = RegisterScanToReferences(new_scan, transaction);

  if (num_new_measurements == 0) {
    // if no constraints were added for this scan, add scan to unregistered list
    ROS_DEBUG(
        "No constraints added to new scan with stamp %d.%d, adding scan to "
        "unregistered list.",
        new_scan.Stamp().sec, new_scan.Stamp().nsec);
    unregistered_clouds_.push_back(new_scan);
  } else {
    ROS_DEBUG("Adding %d measurements to scan with stamp %d.%d",
              num_new_measurements, new_scan.Stamp().sec,
              new_scan.Stamp().nsec);
    // add cloud to reference cloud list
    reference_clouds_.push_front(new_scan);

    // add pose variables for new scan
    transaction.AddPoseVariables(new_scan.Position(), new_scan.Orientation(),
                                 new_scan.Stamp());
  }

  return transaction;
}

void MultiScanRegistrationBase::AddFirstScan(
    const ScanPose& scan,
    bs_constraints::Pose3DStampedTransaction& transaction) {
  ROS_DEBUG("Adding first scan to reference scans.");
  // BEAM_DEBUG("Adding first scan to reference scans.");
  reference_clouds_.push_front(scan);

  // add pose variables for new scan
  transaction.AddPoseVariables(scan.Position(), scan.Orientation(),
                               scan.Stamp());

  if (params_.fix_first_scan) {
    // add prior
    transaction.AddPosePrior(scan.Position(), scan.Orientation(),
                             pose_prior_noise_, "FIRSTSCANPRIOR");
  }

  if (!params_.disable_lidar_map) {
    map_.AddPointCloud(scan.Cloud(), scan.LoamCloud(), scan.Stamp(),
                       scan.T_REFFRAME_LIDAR());
  }

  return;
}

int MultiScanRegistrationBase::RegisterScanToReferences(
    const ScanPose& new_scan,
    bs_constraints::Pose3DStampedTransaction& transaction) {
  if (!params_.GetBaseParams().save_path.empty()) {
    current_scan_path_ =
        beam::CombinePaths(params_.GetBaseParams().save_path,
                           std::to_string(new_scan.Stamp().toSec()));
    boost::filesystem::create_directory(current_scan_path_);
  }

  int counter = 0;
  int num_constraints = 0;

  // open output file
  std::ofstream measurements_file;
  if (!params_.GetBaseParams().save_path.empty()) {
    measurements_file.open(current_scan_path_ +
                           "absolute_pose_measurements.txt");
    measurements_file << "New Scan Stamp: " << new_scan.Stamp().sec << "."
                      << new_scan.Stamp().nsec << "\n\n";
  }

  std::vector<Eigen::Matrix4d, beam::AlignMat4d> lidar_poses_est;
  for (auto ref_iter = reference_clouds_.begin();
       ref_iter != reference_clouds_.end(); ref_iter++) {
    counter++;

    // BEAM_DEBUG("Matching against neighbor no. {}", counter);
    ROS_DEBUG("Matching against neighbor no. %d", counter);

    // run matcher to get refined cloud pose
    Eigen::Matrix4d T_LIDARREF_LIDARTGT;
    if (!MatchScans(*ref_iter, new_scan, T_LIDARREF_LIDARTGT)) { continue; }

    // keep track of all results so that we can average the transform for the
    // lidar map
    if (!params_.disable_lidar_map) {
      Eigen::Matrix4d T_WORLD_LIDARREF = (*ref_iter).T_REFFRAME_LIDAR();
      Eigen::Matrix4d T_WORLD_LIDARCURRENT =
          T_WORLD_LIDARREF * T_LIDARREF_LIDARTGT;
      lidar_poses_est.push_back(T_WORLD_LIDARCURRENT);
    }

    /**
     * We need to convert the relative poses measurements from lidar (or cloud)
     * frames to baselink frames:
     *
     * T_BASELINKREF_BASELINKNEW =
     *    T_BASELINKREF_LIDARREF * T_LIDARREF_LIDARNEW * T_LIDARNEW_BASELINKNEW
     */
    Eigen::Matrix4d T_BASELINKREF_BASELINKNEW = ref_iter->T_BASELINK_LIDAR() *
                                                T_LIDARREF_LIDARTGT *
                                                new_scan.T_LIDAR_BASELINK();
    fuse_variables::Position3DStamped position_relative;
    fuse_variables::Orientation3DStamped orientation_relative;
    bs_common::EigenTransformToFusePose(
        T_BASELINKREF_BASELINKNEW, position_relative, orientation_relative);

    if (!params_.GetBaseParams().save_path.empty()) {
      // calculate measured pose of target (new scan)
      Eigen::Matrix4d T_REFFRAME_BASELINKNEW =
          ref_iter->T_REFFRAME_BASELINK() * T_BASELINKREF_BASELINKNEW;

      // convert to quaternion
      Eigen::Matrix3d R = T_REFFRAME_BASELINKNEW.block(0, 0, 3, 3);
      Eigen::Quaterniond q(R);

      // add measurement to file
      measurements_file << "Ref Scan No. " << counter << ":\n"
                        << "stamp: " << new_scan.Stamp().sec << "."
                        << new_scan.Stamp().nsec << "\n"
                        << "translation: " << T_REFFRAME_BASELINKNEW(0, 3)
                        << ", " << T_REFFRAME_BASELINKNEW(1, 3) << ", "
                        << T_REFFRAME_BASELINKNEW(2, 3) << "\n"
                        << "orientation: " << q.x() << ", " << q.y() << ", "
                        << q.z() << ", " << q.w() << "\n\n";
    }

    // add measurement to transaction
    transaction.AddPoseConstraint(
        ref_iter->Position(), new_scan.Position(), ref_iter->Orientation(),
        new_scan.Orientation(), position_relative, orientation_relative,
        covariance_weight_ * covariance_, source_);

    num_constraints++;
  }

  // close output file
  if (!params_.GetBaseParams().save_path.empty()) { measurements_file.close(); }

  // calculate average and add to lidar map
  if (!params_.disable_lidar_map) {
    Eigen::Matrix4d T_WORLD_LIDAR_AVG =
        beam::AverageTransforms(lidar_poses_est);
    map_.AddPointCloud(new_scan.Cloud(), new_scan.LoamCloud(), new_scan.Stamp(),
                       T_WORLD_LIDAR_AVG);
  }

  return num_constraints;
}

void MultiScanRegistrationBase::InsertCloudInReferences(const ScanPose& scan) {
  // iterate through list and insert when the new scan has a timestamp less than
  // the current scan in reference scans
  for (auto iter = reference_clouds_.begin(); iter != reference_clouds_.end();
       iter++) {
    if (scan.Stamp() > iter->Stamp()) { continue; }

    // insert and then check size of references
    reference_clouds_.insert(iter, scan);
    while (reference_clouds_.size() >= params_.num_neighbors) {
      reference_clouds_.pop_back();
    }

    return;
  }

  // if all scans were prior to new scan, or list was empty, then push to back
  reference_clouds_.push_back(scan);
}

void MultiScanRegistrationBase::UpdateScanPoses(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  for (auto iter = reference_clouds_.begin(); iter != reference_clouds_.end();
       iter++) {
    iter->UpdatePose(graph_msg);
  }
}

void MultiScanRegistrationBase::CleanUpScanLists(
    const ros::Time& new_scan_time) {
  while (reference_clouds_.size() > params_.num_neighbors) {
    reference_clouds_.pop_back();
  }

  while (unregistered_clouds_.size() > max_unregistered_clouds_) {
    unregistered_clouds_.pop_back();
  }

  if (params_.lag_duration == 0) { return; }

  // clear old reference clouds
  auto i = reference_clouds_.begin();
  while (i != reference_clouds_.end()) {
    // remove scan if new_scan_time - ref_scan_time > lag_duration
    if (new_scan_time - i->Stamp() > ros::Duration(params_.lag_duration)) {
      reference_clouds_.erase(i++);
    } else {
      ++i;
    }
  }

  // clear old unregistred clouds
  auto j = unregistered_clouds_.begin();
  while (j != unregistered_clouds_.end()) {
    // remove scan if new_scan_time - unregistered scan time > lag_duration
    if (new_scan_time - j->Stamp() > ros::Duration(params_.lag_duration)) {
      unregistered_clouds_.erase(j++);
    } else {
      ++j;
    }
  }
}

void MultiScanRegistrationBase::RemoveMissingScans(
    fuse_core::Graph::ConstSharedPtr graph_msg, bool require_one_update) {
  auto i = reference_clouds_.begin();
  while (i != reference_clouds_.end()) {
    // first, check that number of updates is greater than 0
    if (require_one_update && i->Updates() == 0) {
      ++i;
      continue;
    }

    // check if variables exist
    bool keep_scan = (graph_msg->variableExists(i->Position().uuid()) &&
                      graph_msg->variableExists(i->Orientation().uuid()));

    // remove variable is required, and update iterator
    if (!keep_scan) {
      reference_clouds_.erase(i++);
    } else {
      ++i;
    }
  }
}

ScanPose MultiScanRegistrationBase::GetScan(const ros::Time& t, bool& success) {
  for (auto iter = Begin(); iter != End(); iter++) {
    if (iter->Stamp() == t) {
      success = true;
      return *iter;
    }
  }
  success = false;
  return ScanPose(PointCloud(), ros::Time(0), Eigen::Matrix4d::Identity());
}

void MultiScanRegistrationBase::PrintScanDetails(std::ostream& stream) {
  for (auto iter = Begin(); iter != End(); iter++) { iter->Print(stream); }
}

void MultiScanRegistrationBase::OutputResults(
    const ScanPose& scan_pose_ref, const ScanPose& scan_pose_tgt,
    const Eigen::Matrix4d& T_LIDARREF_LIDARTGT, bool output_loam_cloud) {
  if (params_.GetBaseParams().save_path.empty()) { return; }

  // get transforms
  Eigen::Matrix4d T_WORLD_LIDARREF = scan_pose_ref.T_REFFRAME_LIDAR();
  Eigen::Matrix4d T_WORLD_LIDARTGT_INIT = scan_pose_tgt.T_REFFRAME_LIDAR();
  Eigen::Matrix4d T_WORLD_LIDARREF_OPT = T_WORLD_LIDARREF * T_LIDARREF_LIDARTGT;

  // get regular clouds
  PointCloud cloud_ref_world;
  PointCloud cloud_tgt_in_world_init;
  PointCloud cloud_tgt_in_world_aligned;

  pcl::transformPointCloud(scan_pose_ref.Cloud(), cloud_ref_world,
                           T_WORLD_LIDARREF);
  pcl::transformPointCloud(scan_pose_tgt.Cloud(), cloud_tgt_in_world_init,
                           T_WORLD_LIDARTGT_INIT);
  pcl::transformPointCloud(scan_pose_tgt.Cloud(), cloud_tgt_in_world_aligned,
                           T_WORLD_LIDARREF_OPT);

  PointCloudCol cloud_ref_world_col =
      beam::ColorPointCloud(cloud_ref_world, 0, 0, 255);
  PointCloudCol cloud_tgt_in_world_init_col =
      beam::ColorPointCloud(cloud_tgt_in_world_init, 255, 0, 0);
  PointCloudCol cloud_tgt_in_world_aligned_col =
      beam::ColorPointCloud(cloud_tgt_in_world_aligned, 0, 255, 0);

  cloud_ref_world_col = beam::AddFrameToCloud(cloud_ref_world_col, coord_frame_,
                                              T_WORLD_LIDARREF);
  cloud_tgt_in_world_init_col = beam::AddFrameToCloud(
      cloud_tgt_in_world_init_col, coord_frame_, T_WORLD_LIDARTGT_INIT);
  cloud_tgt_in_world_aligned_col = beam::AddFrameToCloud(
      cloud_tgt_in_world_aligned_col, coord_frame_, T_WORLD_LIDARREF_OPT);

  // create directories
  double t = scan_pose_ref.Stamp().toSec();
  std::string filename = current_scan_path_ + std::to_string(t);
  boost::filesystem::create_directory(filename + "_ref/");
  boost::filesystem::create_directory(filename + "_tgt_init/");
  boost::filesystem::create_directory(filename + "_tgt_alig/");

  // save clouds
  BEAM_INFO("Saving scan registration results to {}", filename);

  std::string error_message{};
  if (!beam::SavePointCloud<pcl::PointXYZRGB>(
          filename + "_ref.pcd", cloud_ref_world_col,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }
  if (!beam::SavePointCloud<pcl::PointXYZRGB>(
          filename + "_tgt_init.pcd", cloud_tgt_in_world_init_col,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }
  if (!beam::SavePointCloud<pcl::PointXYZRGB>(
          filename + "_tgt_alig.pcd", cloud_tgt_in_world_aligned_col,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }

  if (!output_loam_cloud) { return; }

  // get loam clouds
  LoamPointCloud loam_cloud_ref_world(scan_pose_ref.LoamCloud(),
                                      T_WORLD_LIDARREF);
  LoamPointCloud loam_cloud_tgt_in_world_init(scan_pose_tgt.LoamCloud(),
                                              T_WORLD_LIDARTGT_INIT);
  LoamPointCloud loam_cloud_tgt_in_world_aligned(scan_pose_tgt.LoamCloud(),
                                                 T_WORLD_LIDARREF_OPT);

  loam_cloud_ref_world.SaveCombined(filename + "_ref/", "loam");
  loam_cloud_tgt_in_world_init.SaveCombined(filename + "_tgt_init/",
                                            "loam.pcd");
  loam_cloud_tgt_in_world_aligned.SaveCombined(filename + "_tgt_alig/",
                                               "loam.pcd");
}

MultiScanRegistration::MultiScanRegistration(
    std::unique_ptr<Matcher<PointCloudPtr>> matcher,
    const ScanRegistrationParamsBase& base_params, int num_neighbors,
    double lag_duration, bool disable_lidar_map)
    : MultiScanRegistrationBase(base_params, num_neighbors, lag_duration,
                                disable_lidar_map),
      matcher_(std::move(matcher)) {}

bool MultiScanRegistration::MatchScans(const ScanPose& scan_pose_ref,
                                       const ScanPose& scan_pose_tgt,
                                       Eigen::Matrix4d& T_LIDARREF_LIDARTGT) {
  Eigen::Matrix4d T_LidarRefEst_LidarTgt =
      beam::InvertTransform(scan_pose_ref.T_REFFRAME_LIDAR()) *
      scan_pose_tgt.T_REFFRAME_LIDAR();

  if (!PassedMotionThresholds(T_LidarRefEst_LidarTgt)) { return false; }

  // transform tgt cloud into est ref frame
  PointCloud tgtcloud_in_ref_est_frame;
  pcl::transformPointCloud(scan_pose_tgt.Cloud(), tgtcloud_in_ref_est_frame,
                           T_LidarRefEst_LidarTgt);

  // match clouds
  matcher_->SetRef(std::make_shared<PointCloud>(scan_pose_ref.Cloud()));
  matcher_->SetTarget(std::make_shared<PointCloud>(tgtcloud_in_ref_est_frame));
  if (!matcher_->Match()) {
    BEAM_WARN(
        "Failed scan matching within matcher class. Skipping measurement.");
    return false;
  }

  Eigen::Matrix4d T_RefEst_Ref = matcher_->GetResult().matrix();
  T_LIDARREF_LIDARTGT =
      beam::InvertTransform(T_RefEst_Ref) * T_LidarRefEst_LidarTgt;

  OutputResults(scan_pose_ref, scan_pose_tgt, T_LIDARREF_LIDARTGT, false);

  if (!use_fixed_covariance_) {
    BEAM_WARN(
        "Automated covariance estimation not tested, use fixed covariance!");
    covariance_ = matcher_->GetCovariance();
  }

  return registration_validation_.Validate(T_RefEst_Ref, covariance_);
}

MultiScanLoamRegistration::MultiScanLoamRegistration(
    std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher,
    const ScanRegistrationParamsBase& base_params, int num_neighbors,
    double lag_duration, bool disable_lidar_map)
    : MultiScanRegistrationBase(base_params, num_neighbors, lag_duration,
                                disable_lidar_map),
      matcher_(std::move(matcher)) {}

bool MultiScanLoamRegistration::MatchScans(
    const ScanPose& scan_pose_ref, const ScanPose& scan_pose_tgt,
    Eigen::Matrix4d& T_LIDARREF_LIDARTGT) {
  Eigen::Matrix4d T_LidarRefEst_LidarTgt =
      beam::InvertTransform(scan_pose_ref.T_REFFRAME_LIDAR()) *
      scan_pose_tgt.T_REFFRAME_LIDAR();

  if (!PassedMotionThresholds(T_LidarRefEst_LidarTgt)) { return false; }

  std::shared_ptr<LoamPointCloud> tgtcloud_in_ref_est_frame =
      std::make_shared<LoamPointCloud>(scan_pose_tgt.LoamCloud());
  tgtcloud_in_ref_est_frame->TransformPointCloud(T_LidarRefEst_LidarTgt);

  std::shared_ptr<LoamPointCloud> refcloud_in_ref_frame =
      std::make_shared<LoamPointCloud>(scan_pose_ref.LoamCloud());

  // match clouds
  matcher_->SetRef(refcloud_in_ref_frame);
  matcher_->SetTarget(tgtcloud_in_ref_est_frame);
  if (!matcher_->Match()) {
    BEAM_WARN(
        "Failed scan matching within matcher class. Skipping measurement.");
    return false;
  }

  Eigen::Matrix4d T_RefEst_Ref = matcher_->GetResult().matrix();
  T_LIDARREF_LIDARTGT =
      beam::InvertTransform(T_RefEst_Ref) * T_LidarRefEst_LidarTgt;

  OutputResults(scan_pose_ref, scan_pose_tgt, T_LIDARREF_LIDARTGT, true);

  if (!use_fixed_covariance_) { covariance_ = matcher_->GetCovariance(); }

  return registration_validation_.Validate(T_RefEst_Ref, covariance_);
}

}} // namespace bs_models::scan_registration
