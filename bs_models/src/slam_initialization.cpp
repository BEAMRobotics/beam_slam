#include <bs_models/slam_initialization.h>

#include <bs_models/vision/utils.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::SLAMInitialization, fuse_core::SensorModel)

namespace bs_models {

using namespace vision;

SLAMInitialization::SLAMInitialization()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_image_callback_(
          std::bind(&SLAMInitialization::processImage, this, std::placeholders::_1)) {}

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

  // create frame initializer if desired
  if (!params_.frame_initializer_config.empty()) {
    frame_initializer_ = bs_models::frame_initializers::FrameInitializerBase::Create(
        params_.frame_initializer_config);
  }

  // read imu parameters
  nlohmann::json J;
  beam::ReadJson(params_.imu_intrinsics_path, J);
  imu_params_.cov_prior_noise = J["cov_prior_noise"];
  imu_params_.cov_gyro_noise = Eigen::Matrix3d::Identity() * J["cov_gyro_noise"];
  imu_params_.cov_accel_noise = Eigen::Matrix3d::Identity() * J["cov_accel_noise"];
  imu_params_.cov_gyro_bias = Eigen::Matrix3d::Identity() * J["cov_gyro_bias"];
  imu_params_.cov_accel_bias = Eigen::Matrix3d::Identity() * J["cov_accel_bias"];
}

void SLAMInitialization::onStart() {
  // subscribe to topics
  visual_measurement_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Image>(
      ros::names::resolve(params_.visual_measurement_topic), 1000,
      &ThrottledMeasuremenCallback::callback, &throttled_measurement_callback_,
      ros::TransportHints().tcpNoDelay(false));

  imu_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Image>(
      ros::names::resolve(params_.imu_topic), 1000, &ThrottledIMUCallback::callback,
      &throttled_imu_callback_, ros::TransportHints().tcpNoDelay(false));

  lidar_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Image>(
      ros::names::resolve(params_.lidar_topic), 1000, &ThrottledLidarCallback::callback,
      &throttled_lidar_callback_, ros::TransportHints().tcpNoDelay(false));
}

void SLAMInitialization::processMeasurements(const CameraMeasurementMsg::ConstPtr& msg) {
  // put measurements into landmark container
  const auto num_imgs = landmark_container_.GetMeasurementTimes().size();
  for (const auto& lm : msg->landmarks) {
    Eigen::Vector2d landmark(static_cast<double>(lm.pixel_u), static_cast<double>(lm.pixel_v));

    cv::Mat landmark_descriptor =
        beam_cv::Descriptor::CreateDescriptor(lm.descriptor, msg->descriptor_type);

    beam_containers::LandmarkMeasurement lm(msg->header.time, msg->sensor_id, lm.landmark_id,
                                            num_imgs, landmark, landmark_descriptor);
    landmark_container_.Insert(lm);
  }

  // remove first image from container if we are over the limit
  auto img_times = landmark_container_.GetMeasurementTimes();
  if (num_imgs > params_.visual_window_size) {
    landmark_container_.RemoveMeasurementsAtTime(*img_times.begin());
  }

  if (mode_ != InitMode::VISUAL) { return; }

  if (num_imgs < 2) { return; }

  if (bs_models::vision::computeParallax(landmark_container_, *img_times.begin(),
                                         *img_times.rbegin()) < params_.min_parallax) {
    return;
  }

  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get camera to baselink transform.");
    return;
  }

  init_path_ = bs_models::vision::computePathWithVision(landmark_container_, T_cam_baselink_);

  if (initialize()) { stop(); }
}

void SLAMInitialization::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  static ros::Time last_frame_init_pose = ros::Time(0.0);

  imu_buffer_.push(*msg);

  if (mode_ != InitMode::FRAMEINIT) { return; }

  if (!frame_initializer_) { return; }

  if ((msg->header.stamp - last_frame_init_pose).toSec() < params_.frame_init_frequency) { return; }
  last_frame_init_pose = msg->header.stamp;

  Eigen::Matrix4d T_WORLD_BASELINK;
  bool success = frame_initializer_->GetEstimatedPose(T_WORLD_BASELINK, msg->header.time,
                                                      extrinsics_.GetBaselinkFrameId());
  init_path_[msg->header.time.toNSec()] = T_WORLD_BASELINK;

  const auto [first_time, first_pose] = init_path_.begin();
  const auto [current_time, current_pose] = init_path_.rbegin();
  if (beam::PassedMotionThreshold(first_pose, current_pose, 0.0, params_.min_trajectory_length,
                                  true, true, false)) {
    if (initialize()) { stop(); }
  }
}

void SLAMInitialization::processLidar(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  lidar_buffer_.push(*msg);

  if (mode_ != InitMode::LIDAR) { return; }

  // TODO: compute path/add to it

  const auto [first_time, first_pose] = init_path_.begin();
  const auto [current_time, current_pose] = init_path_.rbegin();
  if (beam::PassedMotionThreshold(first_pose, current_pose, 0.0, params_.min_trajectory_length,
                                  true, true, false)) {
    if (initialize()) { stop(); }
  }
}

bool SLAMInitialization::initialize() {
  // prune poses in path that come before any imu messages
  while (*(init_path_.begin()).first < imu_buffer_.front().header.stamp) {
    init_path_.erase(*(init_path_.begin()).first);
  }

  // Estimate imu biases and gravity using the initial path
  Eigen::Vector3d gravity{0, 0, 0};
  Eigen::Vector3d bg{0, 0, 0};
  Eigen::Vector3d ba{0, 0, 0};
  std::vector<Eigen::Vector3d> velocities;
  double scale{1};
  bs_models::ImuPreintegration::Params imu_params_;
  ImuPreintegration::EstimateParameters(init_path_, imu_buffer_, imu_params_, gravity, bg, ba,
                                        velocities, scale);

  if (use_scale_estimate_ && (scale < 0.02 || scale > 1.0)) {
    ROS_FATAL_STREAM("Invalid scale estimate: " << scale << ", shutting down.");
    return false;
  }

  imu_preint_ = std::make_shared<bs_models::ImuPreintegration>(imu_params_, bg, ba);

  if (mode_ == InitMode::VISUAL) { ScaleAndAlignPath(gravity, velocities); }

  AddPosesAndInertialConstraints();

  AddVisualConstraints();

  if (params_.max_optimization_s == 0) { return; }
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 6;
  options.num_linear_solver_threads = 6;
  options.minimizer_type = ceres::TRUST_REGION;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.max_solver_time_in_seconds = params_.max_optimization_s;
  options.max_num_iterations = 100;
  local_graph_->optimize(options);

  SendInitializationGraph();

  return true;
}

void SLAMInitialization::ScaleAndAlignPath(const Eigen::Vector3d& gravity, const double scale,
                                           std::vector<Eigen::Vector3d>& velocities) {
  if (gravit_.isZero(1e-9)) {
    ROS_WARN("Can't align poses to gravity as it has not been estimated yet.");
    return;
  }
  // estimate rotation from estimated gravity to world gravity
  Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(gravity, GRAVITY_WORLD);

  // apply rotation to initial path
  for (auto& [stamp, pose] : init_path_) {
    Eigen::Matrix4d T;
    bs_common::PoseMsgToTransformationMatrix(pose, T);
    Eigen::Quaterniond ori;
    Eigen::Vector3d pos;
    beam::TransformMatrixToQuaternionAndTranslation(T, ori, pos);
    ori = q * ori;
    pos = q * pos;
    pos = scale * pos;
    beam::QuaternionAndTranslationToTransformMatrix(ori, pos, T);
    bs_common::TransformationMatrixToPoseMsg(T, stamp, pose);
  }

  // apply rotation to velocities
  for (auto& vel : velocities) { vel = q * vel; }
}

void SLAMInitialization::AddPosesAndInertialConstraints() {
  auto img_times = landmark_container_.GetMeasurementTimes();

  std::vector<std::pair<uint64_t, Eigen::Matrix4d>> path;
  std::transform(init_path_.begin(), init_path_.end(), path.begin(),
                 [&](const auto& pair) { return pair; });
  bool start_set = false;

  auto process_frame = [&](const auto& pair) {
    auto [stamp, pose] = pair;
    // add imu data to preint for this frame
    while (imu_buffer_.front().header.stamp <= stamp && !imu_buffer_.empty()) {
      imu_preint_->AddToBuffer(imu_buffer_.front());
      imu_buffer_.pop();
    }

    // add pose to graph
    auto pose_transaction = fuse_core::Transaction::make_shared();
    pose_transaction->stamp(stamp);
    visual_map_->AddBaselinkPose(pose, stamp, pose_transaction);
    local_graph_->update(*pose_transaction);

    // get velocity at the given time
    Eigen::Vector3d velocity_estimate;
    if (init_path_.find(stamp) == init_path_.end()) {
      // interpolate velocity if the frame isnt in the init path
      size_t index = 0;
      for (size_t j = 0; j < path.size(); j++) {
        auto [stamp_tmp, pose_tmp] = path[i];
        if (stamp_tmp > stamp) {
          index = i - 1;
          break;
        }
      }
      ros::Time before(0, path[index].first);
      ros::Time after(0, path[index + 1].first);
      velocity_estimate =
          beam::InterpolateVector(velocities[index], before, velocities[index + 1], after, stamp);
    } else {
      velocity_estimate = velocities[i];
    }
    auto velocity = std::make_shared<fuse_variables::VelocityLinear3DStamped>(stamp);
    velocity->x() = velocity_estimate.x();
    velocity->y() = velocity_estimate.y();
    velocity->z() = velocity_estimate.z();

    // get the fuse pose variables
    auto img_orientation = visual_map_->GetOrientation(stamp);
    auto img_position = visual_map_->GetPosition(stamp);

    // Add appropriate imu constraints
    if (!start_set) {
      imu_preint_->SetStart(stamp, img_orientation, img_position, velocity);
      start_set = true;
    }
    auto imu_transaction = imu_preint_->RegisterNewImuPreintegratedFactor(stamp, img_orientation,
                                                                          img_position, velocity);
    local_graph_->update(*imu_transaction);
  };

  if (mode_ == InitMode::VISUAL || img_times.empty()) {
    // if we use visual for initialization, or there are no image measurements at all, then we
    // proceed normally
    for (int i = 0; i < path.size(); i++) { process_frame(path[i]); }
  } else if (mode_ != InitMode::VISUAL && !img_times.empty()) {
    // if there are exisintg image measurements, but the init mode isnt visual, then we need to
    // estimate poses for the images
    for (const auto& stamp : img_times) {
      auto lb = init_path_.lower_bound(stamp);
      auto ub = init_path_.upper_bound(stamp);
      if (lb == init_path_.end() || ub == init_path_.end()) { continue; }
      lb = std::prev(lb);
      auto T_world_baselink_prev = init_path_[*lb];
      auto T_world_baselink_aft = init_path_[*ub];
      auto T_world_baselink = beam::InterpolateTransform(
          T_world_baselink_prev, beam::RosTimeToChrono(*lb), T_world_baselink_aft,
          beam::RosTimeToChrono(*ub), beam::RosTimeToChrono(stamp));

      process_frame(std::make_pair(stamp, T_world_baselink));
    }
  }

  // update visual map with updated graph
  visual_map_->UpdateGraph(local_graph_);
}

size_t SLAMInitialization::AddVisualConstraints() {
  auto img_times = landmark_container_.GetMeasurementTimes();

  std::vector<std::pair<uint64_t, Eigen::Matrix4d>> path;
  std::transform(init_path_.begin(), init_path_.end(), path.begin(),
                 [&](const auto& pair) { return pair; });

  ros::Time start = path[0].first, end = path[path.size() - 1].first;
  size_t num_landmarks = 0;
  // make transaction
  auto landmark_transaction = fuse_core::Transaction::make_shared();
  landmark_transaction->stamp(end);
  // get all landmarks in the window
  std::vector<uint64_t> landmarks = landmakr_container_->GetLandmarkIDsInWindow(start, end);
  for (auto& id : landmarks) {
    fuse_variables::Point3DLandmark::SharedPtr lm = visual_map_->GetLandmark(id);
    if (lm) {
      // if the landmark already exists then add constraint
      for (auto& pose : path) {
        try {
          ros::Time stamp(0.0, pose.first);
          visual_map_->AddVisualConstraint(stamp, id, landmark_container_->GetValue(stamp, id),
                                           landmark_transaction);
        } catch (const std::out_of_range& oor) {}
      }
    } else {
      // otherwise then triangulate then add the constraints
      std::vector<Eigen::Matrix4d, beam::AlignMat4d> T_cam_world_v;
      std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
      std::vector<ros::Time> observation_stamps;
      beam_cv::FeatureTrack track = landmark_container_->GetTrack(id);
      for (auto& m : track) {
        Eigen::Vector2i tmp;
        if (!cam_model_->UndistortPixel(m.value.cast<int>(), tmp)) continue;
        beam::opt<Eigen::Matrix4d> T = visual_map_->GetCameraPose(m.time_point);
        // check if the pose is in the graph (keyframe)
        if (T.has_value()) {
          pixels.push_back(m.value.cast<int>());
          T_cam_world_v.push_back(T.value().inverse());
          observation_stamps.push_back(m.time_point);
        }
      }

      if (T_cam_world_v.size() >= 2) {
        beam::opt<Eigen::Vector3d> point =
            beam_cv::Triangulation::TriangulatePoint(cam_model_, T_cam_world_v, pixels);
        if (point.has_value()) {
          num_landmarks++;
          visual_map_->AddLandmark(point.value(), id, landmark_transaction);
          for (int i = 0; i < observation_stamps.size(); i++) {
            visual_map_->AddVisualConstraint(observation_stamps[i], id,
                                             tracker_->Get(observation_stamps[i], id),
                                             landmark_transaction);
          }
        }
      }
    }
  }
  // send transaction to graph
  local_graph_->update(*landmark_transaction);
  // update visual map with updated graph
  visual_map_->UpdateGraph(local_graph_);

  return num_landmarks;
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
  sendTransaction(transaction);
}

} // namespace bs_models
