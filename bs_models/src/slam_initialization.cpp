#include <bs_models/slam_initialization.h>

#include <beam_cv/geometry/Triangulation.h>
#include <beam_mapping/Poses.h>
#include <beam_utils/utils.h>
#include <bs_models/vision/utils.h>
#include <pluginlib/class_list_macros.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::SLAMInitialization, fuse_core::SensorModel);

namespace bs_models {

using namespace vision;

SLAMInitialization::SLAMInitialization()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_measurement_callback_(
          std::bind(&SLAMInitialization::processMeasurements, this, std::placeholders::_1)),
      throttled_imu_callback_(
          std::bind(&SLAMInitialization::processIMU, this, std::placeholders::_1)),
      throttled_lidar_callback_(
          std::bind(&SLAMInitialization::processLidar, this, std::placeholders::_1)) {}

void SLAMInitialization::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  calibration_params_.loadFromROS();
  params_.loadFromROS(private_node_handle_);

  // Load camera model and Create Map object
  cam_model_ = beam_calibration::CameraModel::Create(calibration_params_.cam_intrinsics_path);
  cam_model_->InitUndistortMap();

  // create optimization graph
  local_graph_ = std::make_shared<fuse_graphs::HashGraph>();

  // create visual map
  visual_map_ = std::make_shared<vision::VisualMap>(cam_model_);

  // create landmark container
  landmark_container_ = std::make_shared<beam_containers::LandmarkContainer>();

  // create frame initializer if desired
  if (!params_.frame_initializer_config.empty()) {
    frame_initializer_ = bs_models::frame_initializers::FrameInitializerBase::Create(
        params_.frame_initializer_config);
  }

  // read imu parameters
  nlohmann::json J;
  beam::ReadJson(calibration_params_.imu_intrinsics_path, J);
  imu_params_.cov_prior_noise = J["cov_prior_noise"];
  imu_params_.cov_gyro_noise = Eigen::Matrix3d::Identity() * J["cov_gyro_noise"];
  imu_params_.cov_accel_noise = Eigen::Matrix3d::Identity() * J["cov_accel_noise"];
  imu_params_.cov_gyro_bias = Eigen::Matrix3d::Identity() * J["cov_gyro_bias"];
  imu_params_.cov_accel_bias = Eigen::Matrix3d::Identity() * J["cov_accel_bias"];

  // set init mode
  if (params_.init_mode == "VISUAL") {
    mode_ = InitMode::VISUAL;
  } else if (params_.init_mode == "LIDAR") {
    mode_ = InitMode::LIDAR;
  } else if (params_.init_mode == "FRAMEINIT") {
    mode_ = InitMode::FRAMEINIT;
  }

  max_landmark_container_size_ = params_.initialization_window_s * params_.camera_hz;
  imu_buffer_size_ = params_.initialization_window_s * params_.imu_hz;
  lidar_buffer_size_ = params_.initialization_window_s * params_.lidar_hz;
}

void SLAMInitialization::onStart() {
  // subscribe to topics
  visual_measurement_subscriber_ = private_node_handle_.subscribe<CameraMeasurementMsg>(
      ros::names::resolve(params_.visual_measurement_topic), 1000,
      &ThrottledMeasurementCallback::callback, &throttled_measurement_callback_,
      ros::TransportHints().tcpNoDelay(false));

  imu_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Imu>(
      ros::names::resolve(params_.imu_topic), 1000, &ThrottledIMUCallback::callback,
      &throttled_imu_callback_, ros::TransportHints().tcpNoDelay(false));

  lidar_subscriber_ = private_node_handle_.subscribe<sensor_msgs::PointCloud2>(
      ros::names::resolve(params_.lidar_topic), 1000, &ThrottledLidarCallback::callback,
      &throttled_lidar_callback_, ros::TransportHints().tcpNoDelay(false));
}

void SLAMInitialization::processMeasurements(const CameraMeasurementMsg::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE("SLAMInitialization received VISUAL measurements: " << msg->header.stamp);
  // put measurements into landmark container
  for (const auto& lm : msg->landmarks) {
    Eigen::Vector2d landmark(static_cast<double>(lm.pixel_u), static_cast<double>(lm.pixel_v));
    cv::Mat landmark_descriptor =
        beam_cv::Descriptor::VectorDescriptorToCvMat({lm.descriptor.data}, msg->descriptor_type);
    beam_containers::LandmarkMeasurement lm_measurement(msg->header.stamp, msg->sensor_id,
                                                        lm.landmark_id, msg->header.seq, landmark,
                                                        landmark_descriptor);
    landmark_container_->Insert(lm_measurement);
  }

  // remove first image from container if we are over the limit
  if (landmark_container_->NumImages() > max_landmark_container_size_) {
    landmark_container_->PopFront();
  }

  if (mode_ != InitMode::VISUAL) { return; }

  if (landmark_container_->NumImages() < 10) { return; }

  if (landmark_container_->ComputeParallax(landmark_container_->FrontTimestamp(),
                                           landmark_container_->BackTimestamp()) <
      params_.min_parallax) {
    return;
  }

  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get camera to baselink transform.");
    return;
  }

  init_path_ =
      bs_models::vision::ComputePathWithVision(cam_model_, landmark_container_, T_cam_baselink_);

  // if we initialize successfully, stop this sensor model
  if (initialize()) { stop(); }
}

void SLAMInitialization::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE("SLAMInitialization received IMU measurements: " << msg->header.stamp);
  static ros::Time last_frame_init_stamp = ros::Time(0.0);

  imu_buffer_.push(*msg);

  if (imu_buffer_.size() > imu_buffer_size_) { imu_buffer_.pop(); }

  if (mode_ != InitMode::FRAMEINIT) { return; }

  if (!frame_initializer_) {
    ROS_ERROR("No frame initializer, cannot initialize. Exiting.");
    throw std::invalid_argument{"No frame initializer, cannot initialize."};
  }

  if ((msg->header.stamp - last_frame_init_stamp).toSec() < params_.frame_init_frequency) {
    return;
  }
  last_frame_init_stamp = msg->header.stamp;

  Eigen::Matrix4d T_WORLD_BASELINK;
  if (!frame_initializer_->GetEstimatedPose(T_WORLD_BASELINK, msg->header.stamp,
                                            extrinsics_.GetBaselinkFrameId())) {
    ROS_WARN("Error getting pose from frame initializer.");
  }
  init_path_[msg->header.stamp.toNSec()] = T_WORLD_BASELINK.inverse();

  const auto [first_time, first_pose] = *init_path_.begin();
  const auto [current_time, current_pose] = *init_path_.rbegin();
  if (beam::PassedMotionThreshold(first_pose, current_pose, 0.0, params_.min_trajectory_length_m,
                                  true, true, false)) {
    if (initialize()) { stop(); }
  }
}

void SLAMInitialization::processLidar(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE("SLAMInitialization received LIDAR measurements: " << msg->header.stamp);
  lidar_buffer_.push(*msg);

  if (lidar_buffer_.size() > lidar_buffer_size_) { lidar_buffer_.pop(); }

  if (mode_ != InitMode::LIDAR) {
    return;
  } else {
    ROS_ERROR("LIDAR initialization not yet implemented, exiting.");
    throw std::invalid_argument{"LIDAR initialization not yet implemented."};
  }
  // TODO: compute path/add to it

  const auto [first_time, first_pose] = *init_path_.begin();
  const auto [current_time, current_pose] = *init_path_.rbegin();
  if (beam::PassedMotionThreshold(first_pose, current_pose, 0.0, params_.min_trajectory_length_m,
                                  true, true, false)) {
    if (initialize()) { stop(); }
  }
}

bool SLAMInitialization::initialize() {
  if (init_path_.size() < 3) {
    ROS_FATAL_STREAM("Initial path estimate too small.");
    return false;
  }

  // prune poses in path that come before any imu messages
  while ((*init_path_.begin()).first < imu_buffer_.front().header.stamp.toNSec()) {
    init_path_.erase((*init_path_.begin()).first);
  }

  // Estimate imu biases and gravity using the initial path
  bs_models::ImuPreintegration::Params imu_params;
  ImuPreintegration::EstimateParameters(init_path_, imu_buffer_, imu_params_, gravity_, bg_, ba_,
                                        velocities_, scale_);

  if (mode_ == InitMode::VISUAL && (scale_ < 0.02 || scale_ > 1.0)) {
    ROS_WARN_STREAM(__func__ << ": Invalid scale estimate: " << scale_);
    return false;
  }

  imu_preint_ = std::make_shared<bs_models::ImuPreintegration>(imu_params, bg_, ba_);

  AlignPathAndVelocities(mode_ == InitMode::VISUAL);

  AddPosesAndInertialConstraints();

  AddVisualConstraints();

  if (params_.max_optimization_s != 0) {
    ROS_INFO_STREAM(__func__ << ": Optimizing fused graph:");
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    local_graph_->optimizeFor(ros::Duration(params_.max_optimization_s), options);
    visual_map_->UpdateGraph(local_graph_);
  }

  SendInitializationGraph();

  if (!params_.output_folder.empty()) { OutputResults(); }

  return true;
}

void SLAMInitialization::AlignPathAndVelocities(bool apply_scale) {
  if (gravity_.isZero(1e-9)) {
    ROS_WARN("Can't align poses to gravity as it has not been estimated yet.");
    return;
  }
  // estimate rotation from estimated gravity to world gravity
  Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(gravity_, GRAVITY_WORLD);

  // apply rotation to initial path
  for (auto& [stamp, pose] : init_path_) {
    Eigen::Quaterniond ori;
    Eigen::Vector3d pos;
    beam::TransformMatrixToQuaternionAndTranslation(pose, ori, pos);
    ori = q * ori;
    pos = q * pos;
    if (apply_scale) { pos = scale_ * pos; }
    beam::QuaternionAndTranslationToTransformMatrix(ori, pos, pose);
  }

  // apply rotation to velocities
  for (auto& vel : velocities_) { vel.second = q * vel.second; }
}

void SLAMInitialization::AddPosesAndInertialConstraints() {
  bool start_set = false;
  auto process_frame = [&](const auto& pair) {
    auto [stamp, T_baselink_world] = pair;
    // add imu data to preint for this frame
    const auto timestamp = beam::NSecToRos(stamp);
    while (imu_buffer_.front().header.stamp <= timestamp && !imu_buffer_.empty()) {
      imu_preint_->AddToBuffer(imu_buffer_.front());
      imu_buffer_.pop();
    }

    // add pose to graph
    auto pose_transaction = fuse_core::Transaction::make_shared();
    pose_transaction->stamp(timestamp);
    visual_map_->AddBaselinkPose(T_baselink_world.inverse(), timestamp, pose_transaction);
    local_graph_->update(*pose_transaction);

    // get velocity at the given time
    Eigen::Vector3d velocity_estimate = velocities_[stamp];
    auto velocity = std::make_shared<fuse_variables::VelocityLinear3DStamped>(timestamp);
    velocity->x() = velocity_estimate.x();
    velocity->y() = velocity_estimate.y();
    velocity->z() = velocity_estimate.z();

    // get the fuse pose variables
    auto img_orientation = visual_map_->GetOrientation(timestamp);
    auto img_position = visual_map_->GetPosition(timestamp);

    // Add appropriate imu constraints
    if (!start_set) {
      imu_preint_->SetStart(timestamp, img_orientation, img_position, velocity);
      start_set = true;
      return;
    }
    auto imu_transaction = imu_preint_->RegisterNewImuPreintegratedFactor(
        timestamp, img_orientation, img_position, velocity);
    local_graph_->update(*imu_transaction);
  };

  // if there are existing image measurements, but the init mode isnt visual, then we need to
  // estimate poses and velocities for img times
  if (mode_ != InitMode::VISUAL && landmark_container_->NumImages() > 0) {
    for (const auto& stamp : landmark_container_->GetMeasurementTimes()) {
      auto lb = init_path_.lower_bound(stamp);
      auto ub = init_path_.upper_bound(stamp);
      if (lb == init_path_.end() || ub == init_path_.end()) { continue; }
      lb = std::prev(lb);

      // interpolate pose at image time
      init_path_[stamp] = beam::InterpolateTransform(
          init_path_.at(lb->first), beam::NSecToRos(lb->first).toSec(), init_path_.at(ub->first),
          beam::NSecToRos(ub->first).toSec(), beam::NSecToRos(stamp).toSec());

      // interpolate velocity at image time
      velocities_[stamp] = beam::InterpolateVector(
          velocities_.at(lb->first), beam::NSecToRos(lb->first).toSec(), velocities_.at(ub->first),
          beam::NSecToRos(ub->first).toSec(), beam::NSecToRos(stamp).toSec());
    }
  }
  // TODO: if mode != LIDAR but we have lidar frames, interpolate as well

  // process each frame in the path
  for (const auto& pair : init_path_) { process_frame(pair); }

  // update visual map with updated graph
  visual_map_->UpdateGraph(local_graph_);
}

void SLAMInitialization::AddVisualConstraints() {
  const auto start = beam::NSecToRos((*init_path_.begin()).first);
  const auto end = beam::NSecToRos((*init_path_.rbegin()).first);

  size_t num_landmarks = 0;

  auto landmark_transaction = fuse_core::Transaction::make_shared();
  landmark_transaction->stamp(end);

  auto process_landmark = [&](const auto& lm_id) {
    // if the landmark doesn't exist, then estimate it
    if (!visual_map_->GetLandmark(lm_id)) {
      std::vector<Eigen::Matrix4d, beam::AlignMat4d> T_cam_world_v;
      std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
      beam_containers::Track track = landmark_container_->GetTrack(lm_id);
      for (auto& m : track) {
        Eigen::Vector2i tmp;
        if (!cam_model_->UndistortPixel(m.value.cast<int>(), tmp)) continue;
        const auto T_world_camera = visual_map_->GetCameraPose(m.time_point);
        // check if the pose is in the graph (keyframe)
        if (T_world_camera.has_value()) {
          pixels.push_back(m.value.cast<int>());
          T_cam_world_v.push_back(T_world_camera.value().inverse());
        }
      }
      if (T_cam_world_v.size() >= 2) {
        const auto point =
            beam_cv::Triangulation::TriangulatePoint(cam_model_, T_cam_world_v, pixels);
        if (point.has_value()) {
          visual_map_->AddLandmark(point.value(), lm_id, landmark_transaction);
          num_landmarks++;
        }
      }
    }

    // add constraints to the landmark
    for (auto& [stamp, T_baselink_world] : init_path_) {
      const auto timestamp = beam::NSecToRos(stamp);
      try {
        visual_map_->AddVisualConstraint(timestamp, lm_id,
                                         landmark_container_->GetValue(timestamp, lm_id),
                                         landmark_transaction);
      } catch (const std::out_of_range& oor) {}
    }
  };

  // get all landmarks in the window and process
  auto landmarks = landmark_container_->GetLandmarkIDsInWindow(start, end);
  std::for_each(landmarks.begin(), landmarks.end(), process_landmark);

  // send transaction to graph
  local_graph_->update(*landmark_transaction);
  // update visual map with updated graph
  visual_map_->UpdateGraph(local_graph_);

  ROS_INFO_STREAM(__func__ << ": Added " << num_landmarks << " visual landmarks to graph.");
}

void SLAMInitialization::SendInitializationGraph() {
  auto transaction = fuse_core::Transaction::make_shared();
  for (auto& var : local_graph_->getVariables()) {
    transaction->addVariable(std::move(var.clone()));
  }

  // add each constraint in the graph
  for (auto& constraint : local_graph_->getConstraints()) {
    transaction->addConstraint(std::move(constraint.clone()));
  }

  // send transaction to fuse optimizer
  sendTransaction(transaction);
}

void SLAMInitialization::OutputResults() {
  ROS_INFO_STREAM(__func__ << ": Outputting initialization results to '" << params_.output_folder
                           << "'");
  pcl::PointCloud<pcl::PointXYZRGB> frame_cloud;
  beam_mapping::Poses poses;
  for (const auto& [stamp, pose] : init_path_) {
    auto timestamp = beam::NSecToRos(stamp);
    auto T = visual_map_->GetBaselinkPose(timestamp);
    if (T.has_value()) {
      if (T.has_value()) { frame_cloud = beam::AddFrameToCloud(frame_cloud, T.value(), 0.001); }
      poses.AddSingleTimeStamp(timestamp);
      poses.AddSinglePose(T.value());
    }
  }

  // create directory if it doesn't exist
  if (!boost::filesystem::exists(params_.output_folder)) {
    boost::filesystem::create_directory(params_.output_folder);
  }

  // output trajectory as a txt file (tx ty tz, qx qy qz qw)
  std::string txt_path =
      beam::CombinePaths({params_.output_folder, "initialization_trajectory.txt"});
  poses.WriteToTXT(txt_path, beam_mapping::format_type::Type2);

  // output trajectory as a point cloud
  std::string pcd_path =
      beam::CombinePaths({params_.output_folder, "initialization_trajectory.pcd"});
  std::string error_message{};
  if (!beam::SavePointCloud<pcl::PointXYZRGB>(pcd_path, frame_cloud,
                                              beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }
}
} // namespace bs_models
