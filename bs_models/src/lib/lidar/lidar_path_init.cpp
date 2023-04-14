#include <bs_models/lidar/lidar_path_init.h>

#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>

#include <beam_matching/Matchers.h>
#include <beam_optimization/CeresParams.h>
#include <beam_utils/pointclouds.h>
#include <beam_utils/se3.h>
#include <beam_utils/time.h>

#include <bs_common/extrinsics_lookup_online.h>

namespace bs_models {

using namespace beam_matching;

LidarPathInit::LidarPathInit(int lidar_buffer_size)
    : lidar_buffer_size_(lidar_buffer_size) {
  // init scan registration
  std::shared_ptr<LoamParams> matcher_params =
      std::make_shared<LoamParams>("DEFAULT_PATH");

  // override iteration since we need this to be fast and scans are very close
  // to each other so iteration isn't necessary
  matcher_params->iterate_correspondences = true;
  matcher_params->max_correspondence_iterations = 3;
  matcher_params->max_correspondence_distance = 0.25;
  matcher_params->max_corner_less_sharp = 10;

  // override ceres config
  beam_optimization::CeresParams ceres_params;
  ceres_params.GetSolverOptionsMutable().max_num_iterations = 15;
  matcher_params->optimizer_params = ceres_params;

  std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher =
      std::make_unique<LoamMatcher>(*matcher_params);

  scan_registration::ScanToMapLoamRegistration::Params reg_params;
  reg_params.map_size = 15;
  reg_params.store_full_cloud = false;
  reg_params.outlier_threshold_trans_m = 0.5;
  reg_params.outlier_threshold_rot_deg = 45;
  reg_params.min_motion_trans_m = 0;
  reg_params.min_motion_rot_deg = 0;
  reg_params.max_motion_trans_m = 10;
  reg_params.fix_first_scan = false;

  scan_registration_ =
      std::make_unique<scan_registration::ScanToMapLoamRegistration>(
          std::move(matcher), reg_params.GetBaseParams(), reg_params.map_size,
          reg_params.store_full_cloud);
  scan_registration_->SetFixedCovariance(0.000001);
  feature_extractor_ = std::make_shared<LoamFeatureExtractor>(matcher_params);

  // get filter params
  nlohmann::json J;
  std::string filepath = bs_common::GetBeamSlamConfigPath() +
                         "registration_config/input_filters.json";

  BEAM_INFO("Reading input filter params from {}", filepath);
  if (!beam::ReadJson(filepath, J)) {
    BEAM_ERROR("Cannot read input filters json, not using any filters.");
  } else {
    nlohmann::json J_filters;
    bool json_valid{true};
    try {
      J_filters = J["filters"];
    } catch (...) {
      BEAM_ERROR(
          "Missing 'filters' param in input filters config file. Not using "
          "filters.");
      std::cout << "Json Dump: " << J.dump() << "\n";
      json_valid = false;
    }
    if (json_valid) {
      input_filter_params_ = beam_filtering::LoadFilterParamsVector(J_filters);
      BEAM_INFO("Loaded {} input filters", input_filter_params_.size());
    }
  }
}

void LidarPathInit::ProcessLidar(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  ROS_DEBUG("Received incoming scan");

  Eigen::Matrix4d T_WORLD_BASELINKLAST{Eigen::Matrix4d::Identity()};
  if (!keyframes_.empty()) {
    T_WORLD_BASELINKLAST = keyframes_.back().T_REFFRAME_BASELINK();
  }

  PointCloud cloud_current = beam::ROSToPCL(*msg);

  PointCloud cloud_filtered = beam_filtering::FilterPointCloud<pcl::PointXYZ>(
      cloud_current, input_filter_params_);

  Eigen::Matrix4d T_BASELINK_LIDAR;
  if (!extrinsics_.GetT_BASELINK_LIDAR(T_BASELINK_LIDAR, msg->header.stamp)) {
    BEAM_WARN("Unable to get imu to lidar transform with time {}.{}",
              msg->header.stamp.sec, msg->header.stamp.nsec);
    return;
  }

  beam::HighResolutionTimer timer;

  // create scan pose
  ScanPose current_scan_pose(cloud_filtered, msg->header.stamp,
                             T_WORLD_BASELINKLAST, T_BASELINK_LIDAR,
                             feature_extractor_);
  ROS_DEBUG("Time to build scan pose: %.5f", timer.elapsedAndRestart());
  bs_constraints::relative_pose::Pose3DStampedTransaction transaction =
      scan_registration_->RegisterNewScan(current_scan_pose);
  ROS_DEBUG("Time to register scan : %.5f", timer.elapsedAndRestart());
  Eigen::Matrix4d T_WORLD_LIDAR;
  bool scan_in_map = scan_registration_->GetMap().GetScanPose(
      current_scan_pose.Stamp(), T_WORLD_LIDAR);

  if (!scan_in_map) { return; }
  current_scan_pose.UpdatePose(T_WORLD_LIDAR *
                               beam::InvertTransform(T_BASELINK_LIDAR));
  keyframes_.push_back(current_scan_pose);
  keyframe_transactions_.emplace(current_scan_pose.Stamp().toNSec(),
                                 transaction);
  while (keyframes_.size() > lidar_buffer_size_) {
    keyframe_transactions_.erase(keyframes_.front().Stamp().toNSec());
    keyframes_.pop_front();
    SetTrajectoryStart();
  }
}

void LidarPathInit::SetTrajectoryStart() {
  auto iter = keyframes_.begin();
  const Eigen::Matrix4d& T_WORLDOLD_KEYFRAME0 = iter->T_REFFRAME_BASELINK();
  Eigen::Matrix4d T_KEYFRAME0_WORLDOLD =
      beam::InvertTransform(T_WORLDOLD_KEYFRAME0);
  iter->UpdatePose(Eigen::Matrix4d::Identity());
  iter++;
  while (iter != keyframes_.end()) {
    const Eigen::Matrix4d& T_WORLDOLD_KEYFRAMEX = iter->T_REFFRAME_BASELINK();
    Eigen::Matrix4d T_KEYFRAME0_KEYFRAMEX =
        T_KEYFRAME0_WORLDOLD * T_WORLDOLD_KEYFRAMEX;
    iter->UpdatePose(T_KEYFRAME0_KEYFRAMEX);
    iter++;
  }
}

void LidarPathInit::OutputResults(const std::string& output_dir) const {
  if (!boost::filesystem::exists(output_dir)) {
    BEAM_ERROR("Output directory does not exist. Not outputting LO Initializer "
               "results.");
    return;
  }

  BEAM_INFO("Saving results to {}", output_dir);

  // create save directory
  std::string save_path = beam::CombinePaths(
      output_dir, beam::ConvertTimeToDate(std::chrono::system_clock::now()));
  boost::filesystem::create_directory(save_path);

  // iterate through all keyframes, update based on graph and save initial and
  // final values
  for (auto iter = keyframes_.begin(); iter != keyframes_.end(); iter++) {
    iter->SaveCloud(save_path, true, true);
  }
}

double LidarPathInit::CalculateTrajectoryLength() const {
  double length{0};
  auto iter = keyframes_.begin();
  Eigen::Vector3d prev_position = iter->T_REFFRAME_BASELINK().block(0, 3, 3, 1);
  iter++;

  while (iter != keyframes_.end()) {
    Eigen::Vector3d current_position =
        iter->T_REFFRAME_BASELINK().block(0, 3, 3, 1);
    Eigen::Vector3d current_motion = current_position - prev_position;
    length += current_motion.norm();
    prev_position = current_position;
    iter++;
  }

  return length;
}

std::map<uint64_t, Eigen::Matrix4d> LidarPathInit::GetPath() const {
  std::map<uint64_t, Eigen::Matrix4d> path;
  for (const ScanPose& scan_pose : keyframes_) {
    path.emplace(scan_pose.Stamp().toNSec(), scan_pose.T_REFFRAME_BASELINK());
  }
  return path;
}

std::unordered_map<uint64_t, LidarTransactionType>
    LidarPathInit::GetTransactions() const {
  return keyframe_transactions_;
}

void LidarPathInit::UpdateRegistrationMap(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  auto& registration_map = scan_registration_->GetMapMutable();
  int counter{0};
  for (ScanPose& p : keyframes_) {
    if (p.UpdatePose(graph_msg)) {
      counter++;
      registration_map.UpdateScan(p.Stamp(), p.T_REFFRAME_BASELINK(), 0, 0);
    }
  }

  if (counter == 0) {
    throw std::runtime_error{
        "registration map contains no scans from graph message"};
  }
}

} // namespace bs_models