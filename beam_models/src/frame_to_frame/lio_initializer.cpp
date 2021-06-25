#include <beam_models/frame_to_frame/lio_initializer.h>

#include <pluginlib/class_list_macros.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

#include <beam_utils/math.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(beam_models::frame_to_frame::LioInitializer,
                       fuse_core::SensorModel)

namespace beam_models {
namespace frame_to_frame {

using namespace beam_matching;

LioInitializer::LioInitializer() : fuse_core::AsyncSensorModel(1) {}

void LioInitializer::onInit() {
  // Read settings from the parameter sever
  params_.loadFromROS(private_node_handle_);

  // subscribe to imu topic
  imu_subscriber_ = private_node_handle_.subscribe(
      params_.imu_topic, 1000, &LioInitializer::processIMU, this);

  // subscribe to lidar topic
  lidar_subscriber_ = private_node_handle_.subscribe(
      params_.lidar_topic, 10, &LioInitializer::processLidar, this);

  // init imu preintegration
  ImuPreintegration::Params imu_preint_params;
  if (params_.nominal_gravity_direction == "-Z") {
    imu_preint_params.gravity = Eigen::Vector3d(0, 0, -GRAVITY);
  } else if (params_.nominal_gravity_direction == "+Z") {
    imu_preint_params.gravity = Eigen::Vector3d(0, 0, GRAVITY);
  } else if (params_.nominal_gravity_direction == "-X") {
    imu_preint_params.gravity = Eigen::Vector3d(-GRAVITY, 0, 0);
  } else if (params_.nominal_gravity_direction == "+X") {
    imu_preint_params.gravity = Eigen::Vector3d(GRAVITY, 0, 0);
  } else if (params_.nominal_gravity_direction == "-Y") {
    imu_preint_params.gravity = Eigen::Vector3d(0, -GRAVITY, 0);
  } else if (params_.nominal_gravity_direction == "+Y") {
    imu_preint_params.gravity = Eigen::Vector3d(0, GRAVITY, 0);
  } else {
    ROS_ERROR(
        "Invalid nominal_gravity_direction params. Options: -Z, +Z, -Y, +Y, "
        "-X, +X. Using default: -Z");
    imu_preint_params.gravity = Eigen::Vector3d(0, 0, -GRAVITY);
  }
  imu_preintegration_ = std::make_unique<ImuPreintegration>(imu_preint_params);

  // initial scan registration
  std::shared_ptr<LoamParams> matcher_params =
      std::make_shared<LoamParams>(params_.matcher_params_path);
  matcher_ = std::make_unique<LoamMatcher>(*matcher_params);
  std::unique_ptr<beam_matching::Matcher<LoamPointCloudPtr>> matcher =
      std::make_unique<LoamMatcher>(*matcher_params);
  ScanToMapLoamRegistration::Params params;
  params.LoadFromJson(params_.registration_config_path);
  scan_registration_ =
      std::make_unique<ScanToMapLoamRegistration>(std::move(matcher), params);
  feature_extractor_ = std::make_shared<LoamFeatureExtractor>(matcher_params);

  // set covariance if not set to zero in config
  if (std::accumulate(params_.matcher_noise_diagonal.begin(),
                      params_.matcher_noise_diagonal.end(), 0.0) > 0) {
    Eigen::Matrix<double, 6, 6> covariance;
    covariance.setIdentity();
    for (int i = 0; i < 6; i++) {
      covariance(i, i) = params_.matcher_noise_diagonal[i];
    }
    scan_registration_->SetFixedCovariance(covariance);
  } else {
    scan_registration_->SetFixedCovariance(params_.matcher_noise);
  }

  // if outputting scans, clear folder
  if (!params_.scan_output_directory.empty()) {
    if (boost::filesystem::is_directory(params_.scan_output_directory)) {
      boost::filesystem::remove_all(params_.scan_output_directory);
    }
    boost::filesystem::create_directory(params_.scan_output_directory);
  }

  // init graph
  graph_ = std::make_shared<fuse_graphs::HashGraph>();
}

void LioInitializer::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  if (initialization_complete_) {
    return;
  }
  imu_preintegration_->AddToBuffer(*msg);
}

void LioInitializer::processLidar(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (initialization_complete_) {
    return;
  }

  ROS_DEBUG("Received incoming scan");
  PointCloudPtr cloud_current = beam::ROSToPCL(*msg);

  // init first message
  if (keyframe_start_time_.toSec() == 0) {
    imu_preintegration_->SetStart(msg->header.stamp);
    AddPointcloudToKeyframe(*cloud_current, msg->header.stamp);
    keyframe_start_time_ = msg->header.stamp;
    T_WORLD_KEYFRAME_ = imu_preintegration_->GetPose(msg->header.stamp);
    return;
  }

  if (msg->header.stamp - keyframe_start_time_ > params_.aggregation_time) {
    ProcessCurrentKeyframe();
    keyframe_cloud_.clear();
    AddPointcloudToKeyframe(*cloud_current, msg->header.stamp);
    keyframe_start_time_ = msg->header.stamp;
    T_WORLD_KEYFRAME_ = imu_preintegration_->GetPose(msg->header.stamp);
  }

  AddPointcloudToKeyframe(*cloud_current, msg->header.stamp);
}

void LioInitializer::onStop() {}

void LioInitializer::ProcessCurrentKeyframe() {
  // create scan pose
  beam_common::ScanPose current_scan_pose(keyframe_start_time_,
                                          T_WORLD_KEYFRAME_, keyframe_cloud_,
                                          feature_extractor_);

  // first, check if this is the first scan, if so then add the scan and return
  if (keyframes_.empty()) {
    keyframes_.push_back(current_scan_pose);
    return;
  }

  // register current keyframe to last keyframe
  Eigen::Matrix4d T_CLOUD1_CLOUD2;
  if (!MatchScans(keyframes_.back(), current_scan_pose, T_CLOUD1_CLOUD2)) {
    return;
  }

  // update pose of the keyframe and add to list of scan poses
  Eigen::Matrix4d T_WORLD_CLOUD2_Updated =
      keyframes_.back().T_REFFRAME_CLOUD() * T_CLOUD1_CLOUD2;
  current_scan_pose.Update(T_WORLD_CLOUD2_Updated);
  keyframes_.push_back(current_scan_pose);

  // check if time window is full, if not then keep adding to the queue
  if (keyframes_.back().Stamp() - keyframes_.front().Stamp() <
      params_.trajectory_time_window) {
    return;
  }

  // check that trajectory is long enough
  if (CalculateTrajectoryLength() > params_.min_trajectory_distance) {
    // if so, then optimize
    Optimize();
    OutputResults();
    initialization_complete_ = true;
  } else {
    keyframes_.pop_front();
  }
}

bool LioInitializer::MatchScans(const beam_common::ScanPose& scan_pose_1,
                                const beam_common::ScanPose& scan_pose_2,
                                Eigen::Matrix4d& T_CLOUD1_CLOUD2) {
  Eigen::Matrix4d T_CLOUD1_CLOUD2_init =
      beam::InvertTransform(scan_pose_1.T_REFFRAME_CLOUD()) *
      scan_pose_2.T_REFFRAME_CLOUD();

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

  if (!beam::ArePosesEqual(T_CLOUD1_CLOUD2, T_CLOUD1_CLOUD2_init,
                           params_.outlier_threshold_r_deg,
                           params_.outlier_threshold_t_m)) {
    ROS_ERROR(
        "Failed scan matcher transform threshold check. Skipping "
        "lidar keyframe.");
    return false;
  }

  return true;
}

bool LioInitializer::PassedRegThreshold(const Eigen::Matrix4d& T_measured) {
  double t_error = T_measured.block(0, 3, 3, 1).norm();
  Eigen::Matrix3d R = T_measured.block(0, 0, 3, 3);
  double r_error = std::abs(Eigen::AngleAxis<double>(R).angle());

  if (t_error > params_.outlier_threshold_t_m ||
      r_error > params_.outlier_threshold_r_deg) {
    return false;
  }
  return true;
}

void LioInitializer::AddPointcloudToKeyframe(const PointCloud& cloud,
                                             const ros::Time& time) {
  PointCloud cloud_converted;
  Eigen::Matrix4d T_IMU_LIDAR;
  if (!extrinsics_.GetT_IMU_LIDAR(T_IMU_LIDAR, time)) {
    ROS_WARN("Unable to get imu to lidar transform with time $.5f",
             time.toSec());
    return;
  }

  if (time == keyframe_start_time_ || !params_.undistort_scans) {
    pcl::transformPointCloud(cloud, cloud_converted, T_IMU_LIDAR);
  } else {
    Eigen::Matrix4d T_WORLD_IMUNOW = imu_preintegration_->GetPose(time);
    Eigen::Matrix4d T_KEYFRAME_CLOUDNOW =
        beam::InvertTransform(T_WORLD_KEYFRAME_) * T_WORLD_IMUNOW * T_IMU_LIDAR;
    pcl::transformPointCloud(cloud, cloud_converted, T_KEYFRAME_CLOUDNOW);
  }
  keyframe_cloud_ += cloud_converted;
}

double LioInitializer::CalculateTrajectoryLength() {
  double length{0};
  auto iter = keyframes_.begin();
  Eigen::Vector3d prev_position = iter->T_REFFRAME_CLOUD().block(0, 3, 3, 1);
  iter++;

  while (iter != keyframes_.end()) {
    Eigen::Vector3d current_position =
        iter->T_REFFRAME_CLOUD().block(0, 3, 3, 1);
    Eigen::Vector3d current_motion = current_position - prev_position;
    length += current_motion.norm();
    prev_position = current_position;
    iter++;
  }

  return length;
}

void LioInitializer::Optimize() {
  for (auto iter = keyframes_.begin(); iter != keyframes_.end(); iter++) {
    // register scans
    auto reg_transaction = scan_registration_->RegisterNewScan(*iter);
    graph_->update(*reg_transaction.GetTransaction());

    // add imu constraints
    fuse_variables::Orientation3DStamped::SharedPtr orientation =
        std::make_shared<fuse_variables::Orientation3DStamped>(
            iter->Orientation());
    fuse_variables::Position3DStamped::SharedPtr position =
        std::make_shared<fuse_variables::Position3DStamped>(iter->Position());
    auto imu_transaction =
        imu_preintegration_->RegisterNewImuPreintegratedFactor(
            iter->Stamp(), orientation, position);
    graph_->update(*imu_transaction.GetTransaction());
  }
  graph_->optimize();
}

void LioInitializer::OutputResults() {
  if (!boost::filesystem::exists(params_.scan_output_directory)) {
    ROS_ERROR("Output directory does not exist. Not outputting results.");
    return;
  }

  // first, go through all scans and get the initial poses and save those scans
  std::string save_path = params_.scan_output_directory + "/imu_poses/";
  boost::filesystem::create_directory(save_path);
  for (auto iter = keyframes_.begin(); iter != keyframes_.end(); iter++) {
    const Eigen::Matrix4d& T_WORLD_SCAN = iter->T_REFFRAME_CLOUD_INIT();
    PointCloud cloud_world;
    pcl::transformPointCloud(iter->Cloud(), cloud_world, T_WORLD_SCAN);
    pcl::io::savePCDFileASCII(
        save_path + std::to_string(iter->Stamp().toSec()) + ".pcd",
        cloud_world);
  }

  // Next, go through all scans and get the initial poses and save those scans
  save_path = params_.scan_output_directory + "/loam_poses/";
  boost::filesystem::create_directory(save_path);
  for (auto iter = keyframes_.begin(); iter != keyframes_.end(); iter++) {
    const Eigen::Matrix4d& T_WORLD_SCAN = iter->T_REFFRAME_CLOUD();
    PointCloud cloud_world;
    pcl::transformPointCloud(iter->Cloud(), cloud_world, T_WORLD_SCAN);
    pcl::io::savePCDFileASCII(
        save_path + std::to_string(iter->Stamp().toSec()) + ".pcd",
        cloud_world);
  }

  // Finally, update scans with final estimates poses and save
  save_path = params_.scan_output_directory + "/final_poses/";
  boost::filesystem::create_directory(save_path);
  for (auto iter = keyframes_.begin(); iter != keyframes_.end(); iter++) {
    iter->Update(graph_);
    const Eigen::Matrix4d& T_WORLD_SCAN = iter->T_REFFRAME_CLOUD();
    PointCloud cloud_world;
    pcl::transformPointCloud(iter->Cloud(), cloud_world, T_WORLD_SCAN);
    pcl::io::savePCDFileASCII(
        save_path + std::to_string(iter->Stamp().toSec()) + ".pcd",
        cloud_world);
  }
}

}  // namespace frame_to_frame
}  // namespace beam_models
