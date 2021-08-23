#include <bs_models/frame_to_frame/scan_registration/multi_scan_registration.h>

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/transaction.h>

#include <beam_matching/Matchers.h>

#include <bs_common/sensor_proc.h>
#include <bs_common/utils.h>
#include <bs_constraints/frame_to_frame/pose_3d_stamped_transaction.h>

namespace bs_models {
namespace frame_to_frame {

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
    std::string default_path = __FILE__;
    size_t start_iter = default_path.find("bs_models");
    size_t end_iter = default_path.size() - start_iter;
    default_path.erase(start_iter, end_iter);
    default_path +=
        "beam_slam_launch/config/registration_config/multi_scan.json";
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

MultiScanRegistrationBase::MultiScanRegistrationBase(const Params& params)
    : params_(params) {
  // if outputting results, clear output folder:
  if (!output_scan_registration_results_) {
    return;
  }

  if (boost::filesystem::is_directory(tmp_output_path_)) {
    boost::filesystem::remove_all(tmp_output_path_);
  }
  boost::filesystem::create_directory(tmp_output_path_);
}

bs_constraints::frame_to_frame::Pose3DStampedTransaction
MultiScanRegistrationBase::RegisterNewScan(const ScanPose& new_scan) {
  bs_constraints::frame_to_frame::Pose3DStampedTransaction transaction(
      new_scan.Stamp());

  // add pose variables for new scan
  transaction.AddPoseVariables(new_scan.Position(), new_scan.Orientation(),
                               new_scan.Stamp());

  // if first scan, add to list then exit
  if (reference_clouds_.empty()) {
    reference_clouds_.push_front(new_scan);
    if (!params_.disable_lidar_map) {
      map_.AddPointCloud(new_scan.Cloud(), new_scan.Stamp(),
                         new_scan.T_REFFRAME_LIDAR());
    }
    if (params_.fix_first_scan) {
      // build covariance
      fuse_core::Matrix6d prior_covariance;
      prior_covariance.setIdentity();
      prior_covariance = prior_covariance * pose_prior_noise_;

      // add prior
      transaction.AddPosePrior(new_scan.Position(), new_scan.Orientation(),
                               prior_covariance, "FIRSTSCANPRIOR");
    }
    return transaction;
  }

  if (output_scan_registration_results_) {
    current_scan_path_ =
        tmp_output_path_ + std::to_string(new_scan.Stamp().toSec()) + "/";
    boost::filesystem::create_directory(current_scan_path_);
  }

  RemoveOldScans(new_scan.Stamp());

  // for building a lidar map with multi scan registration, we will average the
  // pose estimates from each scan registration. This stores the cumulative sum
  // of all DOFs (x, y, z, rx, ry, rz)
  std::vector<double> estimated_scan_poses_sum(6);

  int counter = 0;
  int num_constraints = 0;

  // open output file
  std::ofstream measurements_file;
  if (output_scan_registration_results_) {
    measurements_file.open(current_scan_path_ +
                           "absolute_pose_measurements.txt");
    measurements_file << "New Scan Stamp: " << new_scan.Stamp().sec << "."
                      << new_scan.Stamp().nsec << "\n\n";
  }

  for (auto ref_iter = reference_clouds_.begin();
       ref_iter != reference_clouds_.end(); ref_iter++) {
    counter++;

    ROS_DEBUG("Matching against neighbor no. %d", counter);

    // run matcher to get refined cloud pose
    Eigen::Matrix4d T_LIDARREF_LIDARTGT;
    Eigen::Matrix<double, 6, 6> covariance;
    if (!MatchScans(*ref_iter, new_scan, T_LIDARREF_LIDARTGT, covariance)) {
      continue;
    }

    // keep track of all results so that we can average the transform for the
    // lidar map
    if (!params_.disable_lidar_map) {
      Eigen::Matrix4d T_WORLD_LIDARREF = (*ref_iter).T_REFFRAME_LIDAR();
      Eigen::Matrix4d T_WORLD_LIDARCURRENT =
          T_WORLD_LIDARREF * T_LIDARREF_LIDARTGT;
      Eigen::Vector3d r =
          beam::RToLieAlgebra(T_WORLD_LIDARCURRENT.block(0, 0, 3, 3));
      estimated_scan_poses_sum[0] += T_WORLD_LIDARCURRENT(0, 3);
      estimated_scan_poses_sum[1] += T_WORLD_LIDARCURRENT(1, 3);
      estimated_scan_poses_sum[2] += T_WORLD_LIDARCURRENT(2, 3);
      estimated_scan_poses_sum[3] += r[0];
      estimated_scan_poses_sum[4] += r[1];
      estimated_scan_poses_sum[5] += r[2];
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

    if (output_scan_registration_results_) {
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
        covariance, params_.source);

    num_constraints++;
  }

  // close output file
  if (output_scan_registration_results_) {
    measurements_file.close();
  }

  // calculate average and add to lidar map
  if (!params_.disable_lidar_map) {
    Eigen::Matrix4d T_WORLD_LIDAR_AVG = Eigen::Matrix4d::Identity();
    Eigen::Vector3d r(estimated_scan_poses_sum[3] / num_constraints,
                      estimated_scan_poses_sum[4] / num_constraints,
                      estimated_scan_poses_sum[5] / num_constraints);
    T_WORLD_LIDAR_AVG.block(0, 0, 3, 3) = beam::LieAlgebraToR(r);
    T_WORLD_LIDAR_AVG(0, 3) = estimated_scan_poses_sum[0] / num_constraints;
    T_WORLD_LIDAR_AVG(1, 3) = estimated_scan_poses_sum[1] / num_constraints;
    T_WORLD_LIDAR_AVG(2, 3) = estimated_scan_poses_sum[2] / num_constraints;
    map_.AddPointCloud(new_scan.Cloud(), new_scan.Stamp(), T_WORLD_LIDAR_AVG);
  }

  // if no constraints were added for this scan, send empty transaction (don't
  // add scan to graph)
  if (num_constraints == 0) {
    return bs_constraints::frame_to_frame::Pose3DStampedTransaction(
        new_scan.Stamp());
  }

  // add cloud to reference cloud list and remove last
  if (reference_clouds_.size() == params_.num_neighbors) {
    reference_clouds_.pop_back();
  }
  reference_clouds_.push_front(new_scan);

  return transaction;
}

bool MultiScanRegistrationBase::PassedRegThreshold(
    const Eigen::Matrix4d& T_measured, const Eigen::Matrix4d& T_estimated) {
  Eigen::Matrix3d R1 = T_measured.block(0, 0, 3, 3);
  Eigen::Matrix3d R2 = T_estimated.block(0, 0, 3, 3);

  double t_error =
      (T_measured.block(0, 3, 3, 1) - T_estimated.block(0, 3, 3, 1)).norm();
  double r_error = std::abs(Eigen::AngleAxis<double>(R1).angle() -
                            Eigen::AngleAxis<double>(R2).angle());

  if (t_error > params_.outlier_threshold_t ||
      r_error > params_.outlier_threshold_r) {
    return false;
  }
  return true;
}

void MultiScanRegistrationBase::UpdateScanPoses(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  for (auto iter = reference_clouds_.begin(); iter != reference_clouds_.end();
       iter++) {
    iter->UpdatePose(graph_msg);
  }
}

void MultiScanRegistrationBase::RemoveOldScans(const ros::Time& new_scan_time) {
  if (params_.lag_duration == 0) {
    return;
  }

  auto i = reference_clouds_.begin();
  while (i != reference_clouds_.end()) {
    // remove scan if new_scan_time - ref_scan_time > lag_duration
    if (new_scan_time - i->Stamp() > ros::Duration(params_.lag_duration)) {
      reference_clouds_.erase(i++);
    } else {
      ++i;
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

bool MultiScanRegistrationBase::PassedMotionThresholds(
    const Eigen::Matrix4d& T_LIDARREF_LIDARTGT) {
  // check max translation
  double d_12 = T_LIDARREF_LIDARTGT.block(0, 3, 3, 1).norm();
  if (d_12 > params_.max_motion_trans_m) {
    return false;
  }

  // check min translation
  if (d_12 >= params_.min_motion_trans_m) {
    return true;
  }

  // check rotation
  Eigen::Matrix3d R = T_LIDARREF_LIDARTGT.block(0, 0, 3, 3);
  if (Eigen::AngleAxis<double>(R).angle() >= params_.min_motion_rot_rad) {
    return true;
  }
  return false;
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
  for (auto iter = Begin(); iter != End(); iter++) {
    iter->Print(stream);
  }
}

void MultiScanRegistrationBase::OutputResults(
    const ScanPose& scan_pose_ref, const ScanPose& scan_pose_tgt,
    const Eigen::Matrix4d& T_LIDARREF_LIDARTGT, bool output_loam_cloud) {
  if (!output_scan_registration_results_) {
    return;
  }

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

  // get loam clouds
  LoamPointCloud loam_cloud_ref_world;
  LoamPointCloud loam_cloud_tgt_in_world_init;
  LoamPointCloud loam_cloud_tgt_in_world_aligned;

  if (output_loam_cloud) {
    loam_cloud_ref_world = scan_pose_ref.LoamCloud();
    loam_cloud_tgt_in_world_init = scan_pose_tgt.LoamCloud();
    loam_cloud_tgt_in_world_aligned = scan_pose_tgt.LoamCloud();

    loam_cloud_ref_world.TransformPointCloud(T_WORLD_LIDARREF);
    loam_cloud_tgt_in_world_init.TransformPointCloud(T_WORLD_LIDARTGT_INIT);
    loam_cloud_tgt_in_world_aligned.TransformPointCloud(T_WORLD_LIDARREF_OPT);
  }

  // create directories
  double t = scan_pose_ref.Stamp().toSec();
  std::string filename = current_scan_path_ + std::to_string(t);
  boost::filesystem::create_directory(filename + "_ref/");
  boost::filesystem::create_directory(filename + "_tgt_init/");
  boost::filesystem::create_directory(filename + "_tgt_alig/");

  // save clouds
  BEAM_INFO("Saving scan registration results to {}", filename);

  pcl::io::savePCDFileASCII(filename + "_ref.pcd", cloud_ref_world_col);
  pcl::io::savePCDFileASCII(filename + "_tgt_init.pcd",
                            cloud_tgt_in_world_init_col);
  pcl::io::savePCDFileASCII(filename + "_tgt_alig.pcd",
                            cloud_tgt_in_world_aligned_col);

  if (output_loam_cloud) {
    loam_cloud_ref_world.Save(filename + "_ref/", true);
    loam_cloud_tgt_in_world_init.Save(filename + "_tgt_init/", true);
    loam_cloud_tgt_in_world_aligned.Save(filename + "_tgt_alig/", true);
  }
}

MultiScanRegistration::MultiScanRegistration(
    std::unique_ptr<Matcher<PointCloudPtr>> matcher, const Params& params)
    : matcher_(std::move(matcher)), MultiScanRegistrationBase(params) {}

bool MultiScanRegistration::MatchScans(
    const ScanPose& scan_pose_ref, const ScanPose& scan_pose_tgt,
    Eigen::Matrix4d& T_LIDARREF_LIDARTGT,
    Eigen::Matrix<double, 6, 6>& covariance) {
  Eigen::Matrix4d T_LidarRefEst_LidarTgt =
      beam::InvertTransform(scan_pose_ref.T_REFFRAME_LIDAR()) *
      scan_pose_tgt.T_REFFRAME_LIDAR();

  if (!PassedMotionThresholds(T_LidarRefEst_LidarTgt)) {
    return false;
  }

  // transform tgt cloud into est ref frame
  PointCloud tgtcloud_in_ref_est_frame;
  pcl::transformPointCloud(scan_pose_tgt.Cloud(), tgtcloud_in_ref_est_frame,
                           T_LidarRefEst_LidarTgt);

  // match clouds
  matcher_->SetRef(std::make_shared<PointCloud>(scan_pose_ref.Cloud()));
  matcher_->SetTarget(std::make_shared<PointCloud>(tgtcloud_in_ref_est_frame));
  if (!matcher_->Match()) {
    BEAM_ERROR("Failed scan matching. Skipping measurement.");
    return false;
  }

  Eigen::Matrix4d T_RefEst_Ref = matcher_->GetResult().matrix();
  T_LIDARREF_LIDARTGT =
      beam::InvertTransform(T_RefEst_Ref) * T_LidarRefEst_LidarTgt;

  OutputResults(scan_pose_ref, scan_pose_tgt, T_LIDARREF_LIDARTGT, false);

  if (!PassedRegThreshold(T_LIDARREF_LIDARTGT, T_LidarRefEst_LidarTgt)) {
    BEAM_ERROR(
        "Failed scan matcher transform threshold check. Skipping "
        "measurement.");
    return false;
  }

  if (use_fixed_covariance_) {
    covariance = covariance_;
  } else {
    BEAM_ERROR(
        "Automated covariance estimation not tested, use fixed covariance!");
    matcher_->EstimateInfo();
    covariance = matcher_->GetInfo();
  }

  return true;
}

MultiScanLoamRegistration::MultiScanLoamRegistration(
    std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher, const Params& params)
    : matcher_(std::move(matcher)), MultiScanRegistrationBase(params) {}

bool MultiScanLoamRegistration::MatchScans(
    const ScanPose& scan_pose_ref, const ScanPose& scan_pose_tgt,
    Eigen::Matrix4d& T_LIDARREF_LIDARTGT,
    Eigen::Matrix<double, 6, 6>& covariance) {
  Eigen::Matrix4d T_LidarRefEst_LidarTgt =
      beam::InvertTransform(scan_pose_ref.T_REFFRAME_LIDAR()) *
      scan_pose_tgt.T_REFFRAME_LIDAR();

  if (!PassedMotionThresholds(T_LidarRefEst_LidarTgt)) {
    return false;
  }

  std::shared_ptr<LoamPointCloud> tgtcloud_in_ref_est_frame =
      std::make_shared<LoamPointCloud>(scan_pose_tgt.LoamCloud());
  tgtcloud_in_ref_est_frame->TransformPointCloud(T_LidarRefEst_LidarTgt);

  std::shared_ptr<LoamPointCloud> refcloud_in_ref_frame =
      std::make_shared<LoamPointCloud>(scan_pose_ref.LoamCloud());

  // match clouds
  matcher_->SetRef(refcloud_in_ref_frame);
  matcher_->SetTarget(tgtcloud_in_ref_est_frame);
  if (!matcher_->Match()) {
    BEAM_ERROR("Failed scan matching. Skipping measurement.");
    return false;
  }

  Eigen::Matrix4d T_RefEst_Ref = matcher_->GetResult().matrix();
  T_LIDARREF_LIDARTGT =
      beam::InvertTransform(T_RefEst_Ref) * T_LidarRefEst_LidarTgt;

  OutputResults(scan_pose_ref, scan_pose_tgt, T_LIDARREF_LIDARTGT, true);

  if (!PassedRegThreshold(T_LIDARREF_LIDARTGT, T_LidarRefEst_LidarTgt)) {
    BEAM_ERROR(
        "Failed scan matcher transform threshold check. Skipping "
        "measurement.");
    return false;
  }

  if (use_fixed_covariance_) {
    covariance = covariance_;
  } else {
    BEAM_ERROR("Must use fixed covariance for loam registration.");
    covariance = covariance_;
  }

  return true;
}

}  // namespace frame_to_frame
}  // namespace bs_models
