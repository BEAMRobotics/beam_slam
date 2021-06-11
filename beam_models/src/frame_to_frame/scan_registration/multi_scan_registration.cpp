#include <beam_models/frame_to_frame/scan_registration/multi_scan_registration.h>

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/transaction.h>

#include <beam_matching/Matchers.h>

#include <beam_common/sensor_proc.h>
#include <beam_common/utils.h>
#include <beam_constraints/frame_to_frame/pose_3d_stamped_transaction.h>

namespace beam_models {
namespace frame_to_frame {

using namespace beam_matching;
using namespace beam_common;

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
    size_t start_iter = default_path.find("beam_models");
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

beam_constraints::frame_to_frame::Pose3DStampedTransaction
MultiScanRegistrationBase::RegisterNewScan(const ScanPose& new_scan) {
  beam_constraints::frame_to_frame::Pose3DStampedTransaction transaction(
      new_scan.Stamp());

  // add pose variables for new scan
  transaction.AddPoseVariables(new_scan.Position(), new_scan.Orientation(),
                               new_scan.Stamp());

  // if first scan, add to list then exit
  if (reference_clouds_.empty()) {
    reference_clouds_.push_front(new_scan);
    if (!params_.disable_lidar_map) {
      map_.AddPointCloud(new_scan.Cloud(), new_scan.Stamp(),
                         new_scan.T_REFFRAME_CLOUD());
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
  for (auto ref_iter = reference_clouds_.begin();
       ref_iter != reference_clouds_.end(); ref_iter++) {
    counter++;

    ROS_DEBUG("Matching against neighbor no. %d", counter);

    // run matcher to get refined cloud pose
    Eigen::Matrix4d T_CLOUDREF_CLOUDCURRENT;
    Eigen::Matrix<double, 6, 6> covariance;
    if (!MatchScans(*ref_iter, new_scan, T_CLOUDREF_CLOUDCURRENT, covariance)) {
      continue;
    }

    // keep track of all results so that we can average the transform for the
    // lidar map
    if (!params_.disable_lidar_map) {
      const Eigen::Matrix4d& T_WORLD_CLOUDREF = (*ref_iter).T_REFFRAME_CLOUD();
      Eigen::Matrix4d T_WORLD_CLOUDCURRENT =
          T_WORLD_CLOUDREF * T_CLOUDREF_CLOUDCURRENT;
      Eigen::Vector3d r =
          beam::RToLieAlgebra(T_WORLD_CLOUDCURRENT.block(0, 0, 3, 3));
      estimated_scan_poses_sum[0] += T_WORLD_CLOUDCURRENT(0, 3);
      estimated_scan_poses_sum[1] += T_WORLD_CLOUDCURRENT(1, 3);
      estimated_scan_poses_sum[2] += T_WORLD_CLOUDCURRENT(2, 3);
      estimated_scan_poses_sum[3] += r[0];
      estimated_scan_poses_sum[4] += r[1];
      estimated_scan_poses_sum[5] += r[2];
    }

    // add measurement to transaction
    fuse_variables::Position3DStamped position_relative;
    fuse_variables::Orientation3DStamped orientation_relative;
    beam_common::EigenTransformToFusePose(
        T_CLOUDREF_CLOUDCURRENT, position_relative, orientation_relative);
    transaction.AddPoseConstraint(
        ref_iter->Position(), new_scan.Position(), ref_iter->Orientation(),
        new_scan.Orientation(), position_relative, orientation_relative,
        covariance, params_.source);

    num_constraints++;
  }

  // calculate average and add to lidar map
  if (!params_.disable_lidar_map) {
    Eigen::Matrix4d T_WORLD_CLOUD_AVG = Eigen::Matrix4d::Identity();
    Eigen::Vector3d r(estimated_scan_poses_sum[3] / num_constraints,
                      estimated_scan_poses_sum[4] / num_constraints,
                      estimated_scan_poses_sum[5] / num_constraints);
    T_WORLD_CLOUD_AVG.block(0, 0, 3, 3) = beam::LieAlgebraToR(r);
    T_WORLD_CLOUD_AVG(0, 3) = estimated_scan_poses_sum[0] / num_constraints;
    T_WORLD_CLOUD_AVG(1, 3) = estimated_scan_poses_sum[1] / num_constraints;
    T_WORLD_CLOUD_AVG(2, 3) = estimated_scan_poses_sum[2] / num_constraints;
    map_.AddPointCloud(new_scan.Cloud(), new_scan.Stamp(), T_WORLD_CLOUD_AVG);
  }

  // if no constraints were added for this scan, send empty transaction (don't
  // add scan to graph)
  if (num_constraints == 0) {
    return beam_constraints::frame_to_frame::Pose3DStampedTransaction(
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
    iter->Update(graph_msg);
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

bool MultiScanRegistrationBase::PassedMinMotion(
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

ScanPose MultiScanRegistrationBase::GetScan(const ros::Time& t, bool& success) {
  for (auto iter = Begin(); iter != End(); iter++) {
    if (iter->Stamp() == t) {
      success = true;
      return *iter;
    }
  }
  success = false;
  return ScanPose(ros::Time(0), Eigen::Matrix4d::Identity(), PointCloud());
}

void MultiScanRegistrationBase::PrintScanDetails(std::ostream& stream) {
  for (auto iter = Begin(); iter != End(); iter++) {
    iter->Print(stream);
  }
}

MultiScanRegistration::MultiScanRegistration(
    std::unique_ptr<Matcher<PointCloudPtr>> matcher, const Params& params)
    : matcher_(std::move(matcher)), MultiScanRegistrationBase(params) {}

bool MultiScanRegistration::MatchScans(
    const ScanPose& scan_pose_1, const ScanPose& scan_pose_2,
    Eigen::Matrix4d& T_CLOUD1_CLOUD2, Eigen::Matrix<double, 6, 6>& covariance) {
  Eigen::Matrix4d T_CLOUD1_CLOUD2_init =
      beam::InvertTransform(scan_pose_1.T_REFFRAME_CLOUD()) *
      scan_pose_2.T_REFFRAME_CLOUD();

  if (!PassedMinMotion(T_CLOUD1_CLOUD2_init)) {
    return false;
  }

  // transform cloud2 into cloud1 frame
  PointCloud cloud2_RefFInit;
  pcl::transformPointCloud(scan_pose_2.Cloud(), cloud2_RefFInit,
                           T_CLOUD1_CLOUD2_init);
  matcher_->SetRef(std::make_shared<PointCloud>(cloud2_RefFInit));
  matcher_->SetTarget(std::make_shared<PointCloud>(scan_pose_1.Cloud()));

  // match clouds
  if (!matcher_->Match()) {
    ROS_ERROR("Failed scan matching. Skipping measurement.");
    return false;
  }

  Eigen::Matrix4d T_CLOUD1Est_CLOUD1Ini = matcher_->GetResult().matrix();
  T_CLOUD1_CLOUD2 = T_CLOUD1Est_CLOUD1Ini * T_CLOUD1_CLOUD2_init;

  OutputResults(scan_pose_1, scan_pose_2, T_CLOUD1_CLOUD2);

  if (!PassedRegThreshold(
          T_CLOUD1_CLOUD2,
          beam::InvertTransform(scan_pose_1.T_REFFRAME_CLOUD()) *
              scan_pose_2.T_REFFRAME_CLOUD())) {
    ROS_ERROR(
        "Failed scan matcher transform threshold check. Skipping "
        "measurement.");
    return false;
  }

  if (use_fixed_covariance_) {
    covariance = covariance_;
  } else {
    matcher_->EstimateInfo();
    covariance = matcher_->GetInfo();
  }

  return true;
}

void MultiScanRegistration::OutputResults(
    const ScanPose& scan_pose_1, const ScanPose& scan_pose_2,
    const Eigen::Matrix4d& T_CLOUD1_CLOUD2) {
  if (!output_scan_registration_results_) {
    return;
  }

  const Eigen::Matrix4d& T_WORLD_CLOUDCURRENT_INIT =
      scan_pose_2.T_REFFRAME_CLOUD();
  const Eigen::Matrix4d& T_WORLD_CLOUDREF_INIT = scan_pose_1.T_REFFRAME_CLOUD();
  Eigen::Matrix4d T_WORLD_CLOUDCURRENT_OPT =
      T_WORLD_CLOUDREF_INIT * T_CLOUD1_CLOUD2;

  PointCloud cloud_ref = scan_pose_1.Cloud();
  PointCloud cloud_ref_world;
  PointCloud cloud_cur_initial_world;
  PointCloud cloud_cur_aligned_world;

  pcl::transformPointCloud(cloud_ref, cloud_ref_world, T_WORLD_CLOUDREF_INIT);
  pcl::transformPointCloud(scan_pose_2.Cloud(), cloud_cur_initial_world,
                           T_WORLD_CLOUDCURRENT_INIT);
  pcl::transformPointCloud(scan_pose_2.Cloud(), cloud_cur_aligned_world,
                           T_WORLD_CLOUDCURRENT_OPT);

  PointCloudCol cloud_ref_world_col =
      beam::ColorPointCloud(cloud_ref_world, 0, 0, 255);
  PointCloudCol cloud_cur_initial_world_col =
      beam::ColorPointCloud(cloud_cur_initial_world, 255, 0, 0);
  PointCloudCol cloud_cur_aligned_world_col =
      beam::ColorPointCloud(cloud_cur_aligned_world, 0, 255, 0);

  cloud_ref_world_col = beam::AddFrameToCloud(cloud_ref_world_col, coord_frame_,
                                              T_WORLD_CLOUDREF_INIT);
  cloud_cur_initial_world_col = beam::AddFrameToCloud(
      cloud_cur_initial_world_col, coord_frame_, T_WORLD_CLOUDCURRENT_INIT);
  cloud_cur_aligned_world_col = beam::AddFrameToCloud(
      cloud_cur_aligned_world_col, coord_frame_, T_WORLD_CLOUDCURRENT_OPT);

  double t = scan_pose_1.Stamp().toSec();
  std::string filename = current_scan_path_ + std::to_string(t);

  pcl::io::savePCDFileASCII(filename + "_ref.pcd", cloud_ref_world_col);
  pcl::io::savePCDFileASCII(filename + "_cur_init.pcd",
                            cloud_cur_initial_world_col);
  pcl::io::savePCDFileASCII(filename + "_cur_alig.pcd",
                            cloud_cur_aligned_world_col);

  ROS_INFO("Saved scan registration results to %s", filename.c_str());
}

MultiScanLoamRegistration::MultiScanLoamRegistration(
    std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher, const Params& params)
    : matcher_(std::move(matcher)), MultiScanRegistrationBase(params) {}

bool MultiScanLoamRegistration::MatchScans(
    const ScanPose& scan_pose_1, const ScanPose& scan_pose_2,
    Eigen::Matrix4d& T_CLOUD1_CLOUD2, Eigen::Matrix<double, 6, 6>& covariance) {
  Eigen::Matrix4d T_CLOUD1_CLOUD2_init =
      beam::InvertTransform(scan_pose_1.T_REFFRAME_CLOUD()) *
      scan_pose_2.T_REFFRAME_CLOUD();

  if (!PassedMinMotion(T_CLOUD1_CLOUD2_init)) {
    return false;
  }

  LoamPointCloud cloud2_RefFInit = scan_pose_2.LoamCloud();
  cloud2_RefFInit.TransformPointCloud(T_CLOUD1_CLOUD2_init);
  matcher_->SetRef(std::make_shared<LoamPointCloud>(cloud2_RefFInit));
  matcher_->SetTarget(
      std::make_shared<LoamPointCloud>(scan_pose_1.LoamCloud()));

  // match clouds
  if (!matcher_->Match()) {
    ROS_ERROR("Failed scan matching. Skipping measurement.");
    return false;
  }

  Eigen::Matrix4d T_CLOUD1Est_CLOUD1Ini = matcher_->GetResult().matrix();
  T_CLOUD1_CLOUD2 = T_CLOUD1Est_CLOUD1Ini * T_CLOUD1_CLOUD2_init;

  OutputResults(scan_pose_1, scan_pose_2, T_CLOUD1_CLOUD2);

  if (!PassedRegThreshold(
          T_CLOUD1_CLOUD2,
          beam::InvertTransform(scan_pose_1.T_REFFRAME_CLOUD()) *
              scan_pose_2.T_REFFRAME_CLOUD())) {
    ROS_ERROR(
        "Failed scan matcher transform threshold check. Skipping "
        "measurement.");
    return false;
  }

  if (use_fixed_covariance_) {
    covariance = covariance_;
  } else {
    matcher_->EstimateInfo();
    covariance = matcher_->GetInfo();
  }

  return true;
}

void MultiScanLoamRegistration::OutputResults(
    const ScanPose& scan_pose_1, const ScanPose& scan_pose_2,
    const Eigen::Matrix4d& T_CLOUD1_CLOUD2) {
  if (!output_scan_registration_results_) {
    return;
  }

  LoamPointCloudPtr cloud_ref_world =
      std::make_shared<LoamPointCloud>(scan_pose_1.LoamCloud());
  LoamPointCloudPtr cloud_cur_initial_world =
      std::make_shared<LoamPointCloud>(scan_pose_2.LoamCloud());
  LoamPointCloudPtr cloud_cur_aligned_world =
      std::make_shared<LoamPointCloud>(scan_pose_2.LoamCloud());

  const Eigen::Matrix4d& T_WORLD_CLOUDCURRENT_INIT =
      scan_pose_2.T_REFFRAME_CLOUD();
  const Eigen::Matrix4d& T_WORLD_CLOUDREF_INIT = scan_pose_1.T_REFFRAME_CLOUD();
  Eigen::Matrix4d T_WORLD_CLOUDCURRENT_OPT =
      T_WORLD_CLOUDREF_INIT * T_CLOUD1_CLOUD2;

  cloud_ref_world->TransformPointCloud(T_WORLD_CLOUDREF_INIT);
  cloud_cur_initial_world->TransformPointCloud(T_WORLD_CLOUDCURRENT_INIT);
  cloud_cur_aligned_world->TransformPointCloud(T_WORLD_CLOUDCURRENT_OPT);

  double t = scan_pose_1.Stamp().toSec();
  std::string filename = current_scan_path_ + std::to_string(t);

  ROS_INFO("Saved scan registration results to %s", filename.c_str());
  boost::filesystem::create_directory(filename + "_ref/");
  cloud_ref_world->Save(filename + "_ref/", true);
  boost::filesystem::create_directory(filename + "_cur_init/");
  cloud_cur_initial_world->Save(filename + "_cur_init/", true);
  boost::filesystem::create_directory(filename + "_cur_alig/");
  cloud_cur_aligned_world->Save(filename + "_cur_alig/", true);
}

}  // namespace frame_to_frame
}  // namespace beam_models
