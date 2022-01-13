#include <bs_models/slam_initializers/lo_initializer.h>

#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pluginlib/class_list_macros.h>

#include <beam_optimization/CeresParams.h>
#include <beam_utils/math.h>
#include <beam_utils/pointclouds.h>
#include <beam_utils/time.h>

#include <bs_common/bs_msgs.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::LoInitializer, fuse_core::SensorModel)

namespace bs_models {

using namespace beam_matching;

LoInitializer::LoInitializer() : fuse_core::AsyncSensorModel(1) {}

void LoInitializer::onInit() {
  // Read settings from the parameter sever
  params_.loadFromROS(private_node_handle_);

  // subscribe to lidar topic
  lidar_subscriber_ = private_node_handle_.subscribe(
      params_.lidar_topic, 100, &LoInitializer::processLidar, this);

  // init publisher
  results_publisher_ =
      private_node_handle_.advertise<bs_common::InitializedPathMsg>("result",
                                                                    1000);

  // init scan registration
  std::shared_ptr<LoamParams> matcher_params =
      std::make_shared<LoamParams>(params_.matcher_params_path);

  // override iteration since we need this to be fast and scans are very close
  // to each other so iteration isn't necessary
  matcher_params->iterate_correspondences = false;

  // override ceres config
  beam_optimization::CeresParams ceres_params(params_.ceres_config_path);
  matcher_params->optimizer_params = ceres_params;

  std::unique_ptr<Matcher<LoamPointCloudPtr>> matcher =
      std::make_unique<LoamMatcher>(*matcher_params);

  scan_registration::ScanToMapLoamRegistration::Params reg_params;
  reg_params.LoadFromJson(params_.registration_config_path);
  reg_params.store_full_cloud = false;

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

  ROS_INFO("Reading input filter params from %s", filepath.c_str());
  if (!beam::ReadJson(filepath, J)) {
    ROS_ERROR("Cannot read input filters json, not using any filters.");
  } else {
    nlohmann::json J_filters;
    bool json_valid{true};
    try {
      J_filters = J["filters"];
    } catch (...) {
      ROS_ERROR(
          "Missing 'filters' param in input filters config file. Not using "
          "filters.");
      std::cout << "Json Dump: " << J.dump() << "\n";
      json_valid = false;
    }
    if (json_valid) {
      input_filter_params_ = beam_filtering::LoadFilterParamsVector(J_filters);
      ROS_INFO("Loaded %d input filters", input_filter_params_.size());
    }
  }
}

void LoInitializer::processLidar(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (initialization_complete_) { return; }

  if ((msg->header.stamp - prev_stamp_).toSec() <
      (1.0 / params_.max_frequency)) {
    return;
  }

  ROS_DEBUG("Received incoming scan");

  Eigen::Matrix4d T_WORLD_BASELINKLAST;
  if (keyframes_.empty()) {
    T_WORLD_BASELINKLAST = Eigen::Matrix4d::Identity();
  } else {
    T_WORLD_BASELINKLAST = keyframes_.back().T_REFFRAME_BASELINK();
  }

  PointCloud cloud_current = beam::ROSToPCL(*msg);

  PointCloud cloud_filtered = beam_filtering::FilterPointCloud<pcl::PointXYZ>(
      cloud_current, input_filter_params_);

  // init first message
  if (keyframe_start_time_.toSec() == 0) {
    ROS_DEBUG("Set first scan and imu start time.");
    keyframe_start_time_ = msg->header.stamp;
    keyframe_cloud_ += cloud_filtered;
    keyframe_scan_counter_++;
    prev_stamp_ = msg->header.stamp;
    return;
  }

  ros::Duration current_scan_period = msg->header.stamp - prev_stamp_;

  if (msg->header.stamp - keyframe_start_time_ + current_scan_period >
      params_.aggregation_time) {
    ProcessCurrentKeyframe(msg->header.stamp);
    keyframe_cloud_.clear();
    keyframe_start_time_ = msg->header.stamp;
    keyframe_scan_counter_ = 0;
    T_WORLD_KEYFRAME_ = T_WORLD_BASELINKLAST;
  }

  keyframe_cloud_ += cloud_filtered;
  keyframe_scan_counter_++;

  prev_stamp_ = msg->header.stamp;
}

void LoInitializer::ProcessCurrentKeyframe(const ros::Time& time) {
  beam::HighResolutionTimer timer;
  if (keyframe_cloud_.size() == 0) { return; }

  ROS_DEBUG("Processing keyframe containing %d scans and %d points.",
            keyframe_scan_counter_, keyframe_cloud_.size());

  Eigen::Matrix4d T_BASELINK_LIDAR;
  if (!extrinsics_.GetT_BASELINK_LIDAR(T_BASELINK_LIDAR, time)) {
    ROS_WARN("Unable to get imu to lidar transform with time $.5f",
             time.toSec());
    return;
  }

  // create scan pose
  ScanPose current_scan_pose(keyframe_cloud_, keyframe_start_time_,
                             T_WORLD_KEYFRAME_, T_BASELINK_LIDAR,
                             feature_extractor_);

  scan_registration_->RegisterNewScan(current_scan_pose);
  Eigen::Matrix4d T_WORLD_LIDAR;
  bool scan_in_map = scan_registration_->GetMap().GetScanPose(
      current_scan_pose.Stamp(), T_WORLD_LIDAR);
  if (scan_in_map) {
    current_scan_pose.UpdatePose(T_WORLD_LIDAR *
                                 beam::InvertTransform(T_BASELINK_LIDAR));
    keyframes_.push_back(current_scan_pose);
  } else {
    return;
  }

  ROS_DEBUG("Total time to process keyframe: %.5f", timer.elapsed());

  // check if time window is full, if not then keep adding to the queue
  if (keyframes_.back().Stamp() - keyframes_.front().Stamp() <
      params_.trajectory_time_window) {
    ROS_DEBUG("Time windows not full, continuing to add keyframes.");
    return;
  }
  ROS_DEBUG("Time window is full, checking trajectory length.");

  // check that trajectory is long enough
  double trajectory_length = CalculateTrajectoryLength(keyframes_);
  ROS_DEBUG("Trajectory length of %.3f m was calculated, with %d keyframes.",
            trajectory_length, keyframes_.size());

  if (trajectory_length > params_.min_trajectory_distance) {
    // if so, then optimize
    ROS_INFO("LO trajectory is long enough.");
    SetTrajectoryStart();
    PublishResults();
    OutputResults();
    initialization_complete_ = true;

    // clear lidar map so we can generate a new one during slam (it's a
    // singleton)
    scan_registration_->GetMapMutable().Clear();

    ROS_INFO("LO initialization complete");
  } else {
    keyframes_.pop_front();
  }
}

void LoInitializer::SetTrajectoryStart() {
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

void LoInitializer::OutputResults() {
  if (params_.scan_output_directory.empty()) { return; }

  if (!boost::filesystem::exists(params_.scan_output_directory)) {
    ROS_ERROR("Output directory does not exist. Not outputting LO Initializer "
              "results.");
    return;
  }

  ROS_INFO("Saving results to %s", params_.scan_output_directory.c_str());

  // create save directory
  std::string save_path =
      params_.scan_output_directory +
      beam::ConvertTimeToDate(std::chrono::system_clock::now()) + "/";
  boost::filesystem::create_directory(save_path);

  // iterate through all keyframes, update based on graph and save initial and
  // final values
  for (auto iter = keyframes_.begin(); iter != keyframes_.end(); iter++) {
    iter->SaveCloud(save_path, true, true);
    iter->SaveLoamCloud(save_path, true, true);
  }
}

void LoInitializer::PublishResults() {
  bs_common::InitializedPathMsg msg;

  // get pose variables and add them to the msg
  std::string baselink_frame_id = extrinsics_.GetImuFrameId();
  int counter = 0;
  for (auto iter = keyframes_.begin(); iter != keyframes_.end(); iter++) {
    geometry_msgs::PoseStamped pose;
    bs_common::EigenTransformToPoseStamped(iter->T_REFFRAME_BASELINK(),
                                           iter->Stamp(), counter,
                                           baselink_frame_id, pose);
    msg.poses.push_back(pose);

    counter++;
  }
  results_publisher_.publish(msg);
}

double LoInitializer::CalculateTrajectoryLength(
    const std::list<ScanPose>& keyframes) {
  double length{0};
  auto iter = keyframes.begin();
  Eigen::Vector3d prev_position = iter->T_REFFRAME_BASELINK().block(0, 3, 3, 1);
  iter++;

  while (iter != keyframes.end()) {
    Eigen::Vector3d current_position =
        iter->T_REFFRAME_BASELINK().block(0, 3, 3, 1);
    Eigen::Vector3d current_motion = current_position - prev_position;
    length += current_motion.norm();
    prev_position = current_position;
    iter++;
  }

  return length;
}

} // namespace bs_models
