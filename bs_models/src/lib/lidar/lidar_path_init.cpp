#include <bs_models/lidar/lidar_path_init.h>

#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>

#include <beam_matching/Matchers.h>
#include <beam_optimization/CeresParams.h>
#include <beam_utils/bspline.h>
#include <beam_utils/pointclouds.h>
#include <beam_utils/se3.h>
#include <beam_utils/time.h>

#include <bs_common/extrinsics_lookup_online.h>

namespace bs_models {

using namespace beam_matching;

LidarPathInit::LidarPathInit(int lidar_buffer_size,
                             const std::string& matcher_config,
                             double information_weight)
    : lidar_buffer_size_(lidar_buffer_size) {
  // init scan registration
  std::shared_ptr<LoamParams> matcher_params;
  if (matcher_config.empty()) {
    matcher_params = std::make_shared<LoamParams>("DEFAULT_PATH");
    // set well tested config
    matcher_params->iterate_correspondences = true;
    matcher_params->max_correspondence_iterations = 5;
    matcher_params->max_correspondence_distance = 0.25;
    matcher_params->max_corner_less_sharp = 10;
  } else {
    std::string ceres_config = bs_common::GetAbsoluteConfigPathFromJson(
        matcher_config, "ceres_config");
    matcher_params = std::make_shared<LoamParams>(matcher_config, ceres_config);
  }

  // override ceres config
  beam_optimization::CeresParams ceres_params;
  ceres_params.GetSolverOptionsMutable().max_num_iterations = 40;
  ceres_params.GetSolverOptionsMutable().num_threads =
      std::thread::hardware_concurrency() / 2;
  matcher_params->optimizer_params = ceres_params;

  std::unique_ptr<LoamMatcher> matcher =
      std::make_unique<LoamMatcher>(*matcher_params);

  scan_registration::ScanToMapLoamRegistration::Params reg_params;
  reg_params.map_size = 10;
  reg_params.min_motion_trans_m = 0;
  reg_params.min_motion_rot_deg = 0;
  reg_params.max_motion_trans_m = 10;
  reg_params.fix_first_scan = false;

  scan_registration_ =
      std::make_unique<scan_registration::ScanToMapLoamRegistration>(
          std::move(matcher), reg_params.GetBaseParams(), reg_params.map_size);
  scan_registration_->SetInformationWeight(information_weight);
  feature_extractor_ = std::make_shared<LoamFeatureExtractor>(matcher_params);

  // get filter params
  nlohmann::json J;
  std::string filepath =
      beam::CombinePaths(bs_common::GetBeamSlamConfigPath(),
                         "lidar_filters/input_filters_slam_init.json");

  BEAM_INFO("Reading input filter params from {}", filepath);
  if (!beam::ReadJson(filepath, J)) {
    BEAM_ERROR("Cannot read input filters json, not using any filters.");
    throw std::runtime_error{"unable to load params"};
  }

  beam::ValidateJsonKeysOrThrow({"filters"}, J);
  nlohmann::json J_filters = J["filters"];
  input_filter_params_ = beam_filtering::LoadFilterParamsVector(J_filters);
  BEAM_INFO("Loaded {} input filters", input_filter_params_.size());
}

void LidarPathInit::ProcessLidar(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  ROS_DEBUG("Received incoming scan");

  if (!InitExtrinsics(msg->header.stamp)) { return; }

  beam::HighResolutionTimer timer;
  PointCloud cloud_current = beam::ROSToPCL(*msg);

  PointCloud cloud_filtered = beam_filtering::FilterPointCloud<pcl::PointXYZ>(
      cloud_current, input_filter_params_);

  Eigen::Matrix4d T_WORLD_BASELINK_EST =
      Get_T_WORLD_BASELINKEST(msg->header.stamp);

  // create scan pose
  ScanPose current_scan_pose(cloud_filtered, msg->header.stamp,
                             T_WORLD_BASELINK_EST, T_BASELINK_LIDAR_,
                             feature_extractor_);
  ROS_DEBUG("Time to build scan pose: %.5f", timer.elapsedAndRestart());
  bs_constraints::Pose3DStampedTransaction transaction =
      scan_registration_->RegisterNewScan(current_scan_pose);
  registration_times_.insert(timer.elapsed());
  ROS_DEBUG("Time to register scan : %.5f", timer.elapsedAndRestart());
  Eigen::Matrix4d T_WORLD_LIDAR;
  bool scan_in_map = scan_registration_->GetMap().GetScanPose(
      current_scan_pose.Stamp(), T_WORLD_LIDAR);

  if (!scan_in_map) { return; }
  current_scan_pose.UpdatePose(T_WORLD_LIDAR *
                               beam::InvertTransform(T_BASELINK_LIDAR_));
  keyframes_.push_back(current_scan_pose);
  keyframe_transactions_.emplace(current_scan_pose.Stamp().toNSec(),
                                 transaction);
  while (keyframes_.size() > lidar_buffer_size_) { keyframes_.pop_front(); }
}

Eigen::Matrix4d LidarPathInit::Get_T_WORLD_BASELINKEST(const ros::Time& stamp) {
  Eigen::Matrix4d T_WORLD_BASELINKLAST{Eigen::Matrix4d::Identity()};
  if (!keyframes_.empty()) {
    T_WORLD_BASELINKLAST = keyframes_.back().T_REFFRAME_BASELINK();
  }

  if (!forward_predict_ || keyframes_.size() < min_spline_count_) {
    return T_WORLD_BASELINKLAST;
  }

  std::vector<beam::Pose> poses;
  for (auto it = keyframes_.begin(); it != keyframes_.end(); it++) {
    int64_t tNs = it->Stamp().toNSec();
    beam::Pose p;
    p.T_FIXED_MOVING = it->T_REFFRAME_BASELINK();
    p.timestampInNs = tNs;
    poses.push_back(p);
  }
  beam::BsplineSE3 spline;
  spline.feed_trajectory(poses);

  Eigen::Matrix4d T_WORLD_BASELINK;
  double t = stamp.toSec();
  if (spline.extrapolate(t, T_WORLD_BASELINK)) { return T_WORLD_BASELINK; }
  BEAM_WARN("spline extrapolation failed");
  return T_WORLD_BASELINKLAST;
}

bool LidarPathInit::InitExtrinsics(const ros::Time& stamp) {
  if (extrinsics_initialized_) { return true; }

  if (!extrinsics_.GetT_BASELINK_LIDAR(T_BASELINK_LIDAR_, stamp)) {
    BEAM_WARN("Unable to get imu to lidar transform with time {}.{}", stamp.sec,
              stamp.nsec);
    return false;
  }
  return true;
}

void LidarPathInit::SetTrajectoryStart(const ros::Time& start_time) {
  if (start_time != ros::Time(0)) {
    while (keyframes_.front().Stamp() < start_time) { keyframes_.pop_front(); }
  }
  ros::Time first_keyframe_time = keyframes_.front().Stamp();

  // reset poses in keyframes
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

  // remove all old transactions
  std::vector<std::shared_ptr<bs_variables::Position3D>> static_ps;
  std::vector<std::shared_ptr<bs_variables::Orientation3D>> static_os;
  std::vector<uint64_t> transactions_to_remove;
  std::unordered_set<fuse_core::UUID, fuse_core::uuid::hash> removed_vars;
  for (auto& [timestamp_ns, transaction] : keyframe_transactions_) {
    auto trans = transaction.GetTransaction();

    // first see if extrinsics or static pose variables exist
    auto added_var_range = trans->addedVariables();
    for (auto it = added_var_range.begin(); it != added_var_range.end(); it++) {
      if (it->type() == "bs_variables::Position3D") {
        auto var = bs_variables::Position3D::make_shared();
        *var = dynamic_cast<const bs_variables::Position3D&>(*it);
        static_ps.push_back(var);
      } else if (it->type() == "bs_variables::Orientation3D") {
        auto var = bs_variables::Orientation3D::make_shared();
        *var = dynamic_cast<const bs_variables::Orientation3D&>(*it);
        static_os.push_back(var);
      }
    }

    // next, remove it if any variables exist that are prior to the start
    // keyframe time
    auto timestamps_range = trans->involvedStamps();
    for (auto it = timestamps_range.begin(); it != timestamps_range.end();
         it++) {
      if (*it < first_keyframe_time) {
        // std::cout << "\n\ntransaction stamp: "
        //           << std::to_string(trans->stamp().toSec()) << "\n";
        // std::cout << "ts: " << std::to_string(it->toSec()) << "\n";
        transactions_to_remove.push_back(timestamp_ns);
        auto var_range = trans->addedVariables();
        for (auto var_it = var_range.begin(); var_it != var_range.end();
             var_it++) {
          if (var_it->type() != "bs_variables::Position3D" &&
              var_it->type() != "bs_variables::Orientation3D") {
            removed_vars.emplace(var_it->uuid());
          }
        }
      }
    }
  }

  // some variables may have been removed that are still in a transaction, so
  // let's add those
  for (auto& [timestamp_ns, transaction] : keyframe_transactions_) {
    auto trans = transaction.GetTransaction();
    auto consts_range = trans->addedConstraints();
    for (auto it = consts_range.begin(); it != consts_range.end(); it++) {
      for (const auto& var_uuid : it->variables()) {
        if (removed_vars.find(var_uuid) != removed_vars.end()) {
          transactions_to_remove.push_back(timestamp_ns);
          break;
        }
      }
    }
  }

  for (const auto& t : transactions_to_remove) {
    keyframe_transactions_.erase(t);
  }

  auto first_trans = keyframe_transactions_.begin()->second.GetTransaction();
  for (int i = 0; i < static_ps.size(); i++) {
    first_trans->addVariable(static_ps.at(i));
    first_trans->addVariable(static_os.at(i));
  }
}

void LidarPathInit::OutputResults(const std::string& output_dir) const {
  if (!boost::filesystem::exists(output_dir)) {
    BEAM_ERROR("Output directory does not exist. Not outputting LO Initializer "
               "results.");
    return;
  }

  BEAM_INFO("Saving lidar path init results to {}", output_dir);

  // iterate through all keyframes, update based on graph and save initial and
  // final values
  for (auto iter = keyframes_.begin(); iter != keyframes_.end(); iter++) {
    PointCloud scan_in_map_final;
    PointCloud scan_in_map_initial;
    pcl::transformPointCloud(iter->Cloud(), scan_in_map_final,
                             iter->T_REFFRAME_LIDAR());
    pcl::transformPointCloud(iter->Cloud(), scan_in_map_initial,
                             iter->T_REFFRAME_LIDAR_INIT());
    iter->SaveCloud(output_dir, true, true);
  }
  std::string error_message;
  if (!beam::SavePointCloud<pcl::PointXYZ>(
          beam::CombinePaths(output_dir, "map_optimized.pcd"), map_optimized_,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }
  if (!beam::SavePointCloud<pcl::PointXYZ>(
          beam::CombinePaths(output_dir, "map_registered.pcd"), map_registered_,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }
  if (!beam::SavePointCloud<pcl::PointXYZ>(
          beam::CombinePaths(output_dir, "map_initial.pcd"), map_init_,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }

  // save registration map
  scan_registration_->GetMap().Save(output_dir);
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

std::map<uint64_t, LidarTransactionType> LidarPathInit::GetTransactions() {
  SetTrajectoryStart();
  return keyframe_transactions_;
}

void LidarPathInit::UpdateRegistrationMap(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  auto& registration_map = scan_registration_->GetMapMutable();
  registration_map.SetMapSize(keyframes_.size());
  int counter{0};

  for (ScanPose& p : keyframes_) {
    // update maps
    PointCloud scan_registered;
    pcl::transformPointCloud(p.Cloud(), scan_registered, p.T_REFFRAME_LIDAR());
    map_registered_ += scan_registered;
    PointCloud scan_init;
    pcl::transformPointCloud(p.Cloud(), scan_init, p.T_REFFRAME_LIDAR_INIT());
    map_init_ += scan_init;

    if (p.UpdatePose(graph_msg)) {
      // add updated
      PointCloud scan_optimized;
      pcl::transformPointCloud(p.Cloud(), scan_optimized, p.T_REFFRAME_LIDAR());
      map_optimized_ += scan_optimized;

      // update registration map
      counter++;
      bool in_map =
          registration_map.UpdateScan(p.Stamp(), p.T_REFFRAME_LIDAR(), 0, 0);
      if (!in_map) {
        registration_map.AddPointCloud(p.Cloud(), p.LoamCloud(), p.Stamp(),
                                       p.T_REFFRAME_LIDAR());
      }
    }
  }

  if (counter == 0) {
    throw std::runtime_error{
        "registration map contains no scans from graph message"};
  }
}

double LidarPathInit::GetMeanRegistrationTimeInS() {
  if (registration_times_.empty()) { return 0; }
  double sum = 0;
  for (auto it = registration_times_.begin(); it != registration_times_.end();
       it++) {
    sum += *it;
  }
  return sum / registration_times_.size();
}

double LidarPathInit::GetMaxRegistrationTimeInS() {
  if (registration_times_.empty()) { return 0; }
  return *registration_times_.rbegin();
}

double LidarPathInit::GetMedianRegistrationTimeInS() {
  if (registration_times_.size() == 1) {
    return *registration_times_.begin();
  } else if (registration_times_.empty()) {
    return 0;
  }
  auto iter = registration_times_.begin();
  std::advance(iter, registration_times_.size() / 2);
  return *iter;
}

void LidarPathInit::Reset() {
  keyframes_.clear();
  keyframe_transactions_.clear();
  registration_times_.clear();
  map_optimized_ = PointCloud();
  map_registered_ = PointCloud();
  map_init_ = PointCloud();
  scan_registration_->reset();
}

} // namespace bs_models