#include <bs_models/slam_initialization.h>

#include <fuse_constraints/relative_constraint.h>
#include <fuse_constraints/relative_pose_3d_stamped_constraint.h>
#include <fuse_variables/acceleration_linear_3d_stamped.h>
#include <fuse_variables/velocity_angular_3d_stamped.h>
#include <pluginlib/class_list_macros.h>

#include <beam_cv/geometry/Triangulation.h>
#include <beam_utils/utils.h>

#include <bs_common/visualization.h>
#include <bs_models/graph_visualization/helpers.h>
#include <bs_models/imu/inertial_alignment.h>
#include <bs_models/vision/utils.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::SLAMInitialization, fuse_core::SensorModel);

namespace bs_models {

using namespace vision;

SLAMInitialization::SLAMInitialization()
    : fuse_core::AsyncSensorModel(3),
      device_id_(fuse_core::uuid::NIL),
      throttled_measurement_callback_(
          std::bind(&SLAMInitialization::processCameraMeasurements, this,
                    std::placeholders::_1)),
      throttled_imu_callback_(std::bind(&SLAMInitialization::processIMU, this,
                                        std::placeholders::_1)),
      throttled_lidar_callback_(std::bind(&SLAMInitialization::processLidar,
                                          this, std::placeholders::_1)) {}

void SLAMInitialization::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  calibration_params_.loadFromROS();
  params_.loadFromROS(private_node_handle_);

  // Load camera model and create map object
  cam_model_ = beam_calibration::CameraModel::Create(
      calibration_params_.cam_intrinsics_path);
  cam_intrinsic_matrix_ = cam_model_->GetRectifiedModel()->GetIntrinsicMatrix();
  cv::Mat K(3, 3, CV_32F);
  cv::eigen2cv(cam_intrinsic_matrix_, K);
  K_ = K;

  visual_map_ = std::make_shared<vision::VisualMap>(
      name(), cam_model_, params_.reprojection_loss,
      params_.reprojection_information_weight);

  // create optimization graph
  local_graph_ = std::make_shared<fuse_graphs::HashGraph>();

  // create landmark container
  landmark_container_ = std::make_shared<beam_containers::LandmarkContainer>();

  // create frame initializer if desired
  if (!params_.frame_initializer_config.empty()) {
    frame_initializer_ = std::make_unique<bs_models::FrameInitializer>(
        params_.frame_initializer_config);
  }

  // read imu parameters
  nlohmann::json J;
  BEAM_INFO("loading imu calibration file: {}",
            calibration_params_.imu_intrinsics_path);
  if (!beam::ReadJson(calibration_params_.imu_intrinsics_path, J)) {
    BEAM_CRITICAL("cannot load imu calibration params file: {}",
                  calibration_params_.imu_intrinsics_path);
    throw std::runtime_error{"invalid filepath"};
  }

  beam::ValidateJsonKeysOrThrow({"cov_prior_noise", "cov_gyro_noise",
                                 "cov_accel_noise", "cov_gyro_bias",
                                 "cov_accel_bias"},
                                J);
  imu_params_.cov_prior_noise = J["cov_prior_noise"];
  imu_params_.cov_gyro_noise =
      Eigen::Matrix3d::Identity() * J["cov_gyro_noise"];
  imu_params_.cov_accel_noise =
      Eigen::Matrix3d::Identity() * J["cov_accel_noise"];
  imu_params_.cov_gyro_bias = Eigen::Matrix3d::Identity() * J["cov_gyro_bias"];
  imu_params_.cov_accel_bias =
      Eigen::Matrix3d::Identity() * J["cov_accel_bias"];

  max_landmark_container_size_ =
      params_.initialization_window_s * calibration_params_.camera_hz;
  imu_buffer_size_ =
      params_.initialization_window_s * 2.0 * calibration_params_.imu_hz;

  if (calibration_params_.lidar_hz < 1 / min_lidar_scan_period_s_) {
    lidar_buffer_size_ =
        params_.initialization_window_s * calibration_params_.lidar_hz;
  } else {
    lidar_buffer_size_ =
        params_.initialization_window_s * 1 / min_lidar_scan_period_s_;
  }

  // set init mode
  if (params_.init_mode == "VISUAL") {
    ROS_FATAL(
        "JAKE - Visual not working -> gravity and scale estimate is wrong.");
    mode_ = InitMode::VISUAL;
  } else if (params_.init_mode == "LIDAR") {
    mode_ = InitMode::LIDAR;
    lidar_path_init_ = std::make_unique<LidarPathInit>(
        lidar_buffer_size_, params_.matcher_config,
        params_.lidar_information_weight);
  } else {
    throw std::invalid_argument{"invalid init mode, options: VISUAL, LIDAR"};
  }
}

void SLAMInitialization::onStart() {
  // subscribe to topics
  visual_measurement_subscriber_ =
      private_node_handle_.subscribe<bs_common::CameraMeasurementMsg>(
          ros::names::resolve(params_.visual_measurement_topic), 200,
          &ThrottledMeasurementCallback::callback,
          &throttled_measurement_callback_,
          ros::TransportHints().tcpNoDelay(false));

  imu_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Imu>(
      ros::names::resolve(params_.imu_topic), 2000,
      &ThrottledIMUCallback::callback, &throttled_imu_callback_,
      ros::TransportHints().tcpNoDelay(false));

  lidar_subscriber_ = private_node_handle_.subscribe<sensor_msgs::PointCloud2>(
      ros::names::resolve(params_.lidar_topic), 100,
      &ThrottledLidarCallback::callback, &throttled_lidar_callback_,
      ros::TransportHints().tcpNoDelay(false));
}

void SLAMInitialization::processFrameInit(const ros::Time& timestamp) {
  assert(frame_initializer_ &&
         "No frame initializer, cannot initialize. Exiting.");

  frame_init_buffer_.push_back(timestamp);

  auto front_timestamp = frame_init_buffer_.front();

  Eigen::Matrix4d T_WORLD_BASELINK;
  std::string error_msg;
  if (!frame_initializer_->GetPose(T_WORLD_BASELINK, front_timestamp,
                                   extrinsics_.GetBaselinkFrameId(),
                                   error_msg)) {
    ROS_WARN("Error getting pose from frame initializer, error: %s",
             error_msg.c_str());
    return;
  }
  init_path_[front_timestamp.toNSec()] = T_WORLD_BASELINK;
  frame_init_buffer_.pop_front();

  // check if path is long enough
  const Eigen::Matrix4d& first_pose = init_path_.begin()->second;
  const Eigen::Matrix4d& current_pose = init_path_.rbegin()->second;
  if (!beam::PassedMotionThreshold(first_pose, current_pose, 0.0,
                                   params_.min_trajectory_length_m, true, true,
                                   false)) {
    return;
  }

  // if we initialize successfully, stop this sensor model
  if (Initialize()) { shutdown(); }
}

void SLAMInitialization::processCameraMeasurements(
    const bs_common::CameraMeasurementMsg::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE(
      "SLAMInitialization received VISUAL measurements: " << msg->header.stamp);
  AddMeasurementsToContainer(msg);

  // remove first image from container if we are over the limit
  if (landmark_container_->NumImages() >
      max_landmark_container_size_ + calibration_params_.camera_hz / 2) {
    landmark_container_->PopFront();
  }

  // attempt initialization via frame initializer if its available
  if (frame_initializer_) {
    processFrameInit(msg->header.stamp);
    return;
  }

  if (mode_ != InitMode::VISUAL) { return; }

  // return if the min parallax hasn't been reached
  auto parallax = landmark_container_->ComputeParallax(
      landmark_container_->FrontTimestamp(),
      landmark_container_->BackTimestamp());
  if (parallax < params_.min_visual_parallax) { return; }

  // return if cant get the extrinsic
  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get camera to baselink transform.");
    return;
  }

  // compute visual path if no frame initializer is present
  ROS_INFO("Attempting visual initialization");
  init_path_ = bs_models::vision::ComputePathWithVision(
      cam_model_, landmark_container_, T_cam_baselink_,
      params_.reprojection_loss, 1.0, params_.reprojection_information_weight,
      10.0);

  // if we initialize successfully, stop this sensor model
  if (Initialize()) {
    shutdown();
    return;
  }

  ROS_INFO("Visual initialization, failure, clearing buffer and retrying...");
  // if we don't, prune the first second of images
  if (landmark_container_->NumImages() > calibration_params_.camera_hz) {
    for (int i = 0; i < calibration_params_.camera_hz; i++) {
      landmark_container_->PopFront();
    }
  }
}

void SLAMInitialization::processLidar(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE(
      "SLAMInitialization received LIDAR measurements: " << msg->header.stamp);

  // attempt initialization via frame initializer if its available
  if (frame_initializer_) {
    processFrameInit(msg->header.stamp);
    return;
  }

  if (mode_ != InitMode::LIDAR) { return; }

  double time_in_s = msg->header.stamp.toSec();
  if (time_in_s - last_lidar_scan_time_s_ < min_lidar_scan_period_s_) {
    return;
  }
  last_lidar_scan_time_s_ = time_in_s;

  lidar_path_init_->ProcessLidar(msg);
  double traj_length = lidar_path_init_->CalculateTrajectoryLength();
  if (traj_length < params_.min_trajectory_length_m) { return; }
  BEAM_INFO("trajectory min. length reached, initializing");
  BEAM_INFO("Registration time statistics: mean {}s, median {}s, max {}s",
            lidar_path_init_->GetMeanRegistrationTimeInS(),
            lidar_path_init_->GetMedianRegistrationTimeInS(),
            lidar_path_init_->GetMaxRegistrationTimeInS());

  init_path_ = lidar_path_init_->GetPath();

  beam::HighResolutionTimer timer;
  if (Initialize()) {
    BEAM_INFO("done initialization, total time: {}s", timer.elapsed());
    shutdown();
  }
}

void SLAMInitialization::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE(
      "SLAMInitialization received IMU measurements: " << msg->header.stamp);
  imu_buffer_.push_back(*msg);
  if (imu_buffer_.size() > imu_buffer_size_) { imu_buffer_.pop_front(); }
}

bool SLAMInitialization::Initialize() {
  if (imu_buffer_.empty()) {
    ROS_ERROR_STREAM(__func__ << ": IMU buffer empty, cannot initialize!");
    return false;
  }

  // prune poses in path at start that don't have >= imu messages before it
  auto second_imu_msg = std::next(imu_buffer_.begin());
  if (lidar_path_init_) {
    lidar_path_init_->SetTrajectoryStart(imu_buffer_.begin()->header.stamp);
  }

  while (init_path_.begin()->first < second_imu_msg->header.stamp.toNSec()) {
    init_path_.erase(init_path_.begin()->first);
  }

  if (init_path_.size() < 3) {
    ROS_WARN_STREAM(__func__ << ": Initial path estimate too small.");
    return false;
  }

  // remove any visual measurements that are outside of the init path
  auto visual_measurements = landmark_container_->GetMeasurementTimes();
  for (const auto& stamp : visual_measurements) {
    if (stamp.toNSec() < init_path_.begin()->first) {
      landmark_container_->RemoveMeasurementsAtTime(stamp);
    }
  }

  // Estimate imu biases and gravity using the initial path
  bs_models::imu::EstimateParameters(init_path_, imu_buffer_, imu_params_,
                                     gravity_, bg_, ba_, velocities_, scale_);
  if (!frame_initializer_ && mode_ == InitMode::VISUAL &&
      (scale_ < 0.02 || scale_ > 1.0)) {
    ROS_WARN_STREAM(__func__ << ": Invalid scale estimate: " << scale_);
    return false;
  }

  imu_preint_ = std::make_shared<bs_models::ImuPreintegration>(
      name(), imu_params_, bg_, ba_, params_.inertial_information_weight);

  AlignPathAndVelocities(mode_ == InitMode::VISUAL && !frame_initializer_);
  InterpolateVisualMeasurements();
  AddPosesAndInertialConstraints();
  AddVisualConstraints();
  AddLidarConstraints();

  if (!params_.output_folder.empty()) {
    graph_poses_before_opt_ = bs_common::GetGraphPosesAsCloud(*local_graph_);
    local_graph_->print(graph_before_opt_);
    lidar_constraints_before_opt_ = graph_visualization::
        GetGraphRelativePoseWithExtrinsicsConstraintsAsCloud(*local_graph_,
                                                             "LidarOdometry::");
    imu_constraints_before_opt_ =
        graph_visualization::GetGraphRelativeImuConstraintsAsCloud(
            *local_graph_, 0.01, 0.15);
  }
  if (params_.max_optimization_s > 0.0) {
    ROS_INFO_STREAM(__func__ << ": Optimizing fused initialization graph:");
    ceres::Solver::Options options;
    options.minimizer_type = ceres::TRUST_REGION;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = std::thread::hardware_concurrency() / 2;
    options.max_num_iterations = 1000;
    local_graph_->optimizeFor(ros::Duration(params_.max_optimization_s),
                              options);
    visual_map_->UpdateGraph(*local_graph_);
    if (lidar_path_init_) {
      lidar_path_init_->UpdateRegistrationMap(local_graph_);
    }
  }

  OutputResults();
  SendInitializationGraph();

  return true;
}

void SLAMInitialization::InterpolateVisualMeasurements() {
  if (landmark_container_->NumImages() == 0) { return; }

  // interpolate for existing visual measurements if they aren't in the path
  auto visual_measurements = landmark_container_->GetMeasurementTimes();
  for (const auto& stamp : visual_measurements) {
    auto nsec = stamp.toNSec();
    auto lb = init_path_.lower_bound(nsec);
    auto ub = init_path_.upper_bound(nsec);
    if (lb == init_path_.end() || ub == init_path_.end() || lb->first == nsec) {
      continue;
    }
    lb = std::prev(lb);
    auto ub_time = beam::NSecToRos(ub->first).toSec();
    auto lb_time = beam::NSecToRos(lb->first).toSec();
    // interpolate pose at image time
    init_path_[nsec] = beam::InterpolateTransform(
        init_path_.at(lb->first), lb_time, init_path_.at(ub->first), ub_time,
        stamp.toSec());
    // interpolate velocity at image time
    velocities_[nsec] = beam::InterpolateVector(
        velocities_.at(lb->first), lb_time, velocities_.at(ub->first), ub_time,
        stamp.toSec());
  }
}

void SLAMInitialization::AlignPathAndVelocities(bool apply_scale) {
  Eigen::Matrix4d T_baselink_sensor;
  if (mode_ == InitMode::VISUAL) {
    if (!extrinsics_.GetT_BASELINK_CAMERA(T_baselink_sensor)) {
      ROS_ERROR("Unable to get camera to baselink transform.");
      throw std::runtime_error{"Unable to get camera to baselink transform."};
    }
  } else {
    if (!extrinsics_.GetT_BASELINK_LIDAR(T_baselink_sensor)) {
      ROS_ERROR("Unable to get lidar to baselink transform.");
      throw std::runtime_error{"Unable to get lidar to baselink transform."};
    }
  }

  // estimate rotation from estimated gravity to world gravity
  Eigen::Quaterniond q =
      Eigen::Quaterniond::FromTwoVectors(gravity_, GRAVITY_WORLD);

  // apply rotation to initial path
  for (auto& [stamp, T_WORLD_BASELINK] : init_path_) {
    Eigen::Quaterniond ori;
    Eigen::Vector3d pos;
    beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_BASELINK, ori, pos);
    ori = q * ori;
    pos = q * pos;
    if (apply_scale) { pos = scale_ * pos; }
    beam::QuaternionAndTranslationToTransformMatrix(ori, pos, T_WORLD_BASELINK);
  }

  // apply rotation to velocities
  for (auto& vel : velocities_) { vel.second = q * vel.second; }
}

void SLAMInitialization::AddPosesAndInertialConstraints() {
  // add imu constraints between all poses in the path
  bool start_set = false;
  auto process_frame = [&](const auto& pair) {
    auto [stamp, T_WORLD_BASELINK] = pair;
    const auto timestamp = beam::NSecToRos(stamp);
    auto transaction = fuse_core::Transaction::make_shared();
    transaction->stamp(timestamp);

    // add imu data to preint for this frame
    while (imu_buffer_.front().header.stamp < timestamp &&
           !imu_buffer_.empty()) {
      imu_preint_->AddToBuffer(imu_buffer_.front());
      imu_buffer_.pop_front();
    }

    // add pose and velocity to graph
    visual_map_->AddBaselinkPose(T_WORLD_BASELINK, timestamp, transaction);
    Eigen::Vector3d velocity_estimate = velocities_[stamp];
    auto velocity =
        std::make_shared<fuse_variables::VelocityLinear3DStamped>(timestamp);
    velocity->x() = velocity_estimate.x();
    velocity->y() = velocity_estimate.y();
    velocity->z() = velocity_estimate.z();
    transaction->addVariable(velocity);
    local_graph_->update(*transaction);

    // get the fuse pose variables
    auto img_orientation = visual_map_->GetOrientation(timestamp);
    auto img_position = visual_map_->GetPosition(timestamp);

    // Add appropriate imu constraints
    if (!start_set) {
      imu_preint_->SetStart(timestamp, img_orientation, img_position, velocity);
      start_set = true;
      return;
    }

    // no imu measurements between states -> add zero motion constraint
    if (imu_preint_->CurrentBufferSize() == 0) {
      auto zero_motion_transaction = fuse_core::Transaction::make_shared();
      zero_motion_transaction->stamp(timestamp);

      auto start_state = imu_preint_->GetImuState();
      if (timestamp.toSec() - start_state.Stamp().toSec() > 0.005) {
        ROS_FATAL(
            "Adding zero motion constraint between states that are far apart!");
        throw std::runtime_error(
            "Adding zero motion constraint between states that are far apart!");
      }
      bs_common::ImuState new_state(
          timestamp, start_state.OrientationQuat(), start_state.PositionVec(),
          start_state.VelocityVec(), start_state.GyroBiasVec(),
          start_state.AccelBiasVec());
      bs_common::AddZeroMotionFactor("SLAMINIT", start_state, new_state,
                                     zero_motion_transaction);
      local_graph_->update(*zero_motion_transaction);
      return;
    }

    auto imu_transaction = imu_preint_->RegisterNewImuPreintegratedFactor(
        timestamp, img_orientation, img_position, velocity);
    local_graph_->update(*imu_transaction);
  };

  // process each frame in the path
  std::for_each(init_path_.begin(), init_path_.end(), process_frame);

  // update visual map with updated graph
  visual_map_->UpdateGraph(*local_graph_);
}

void SLAMInitialization::AddVisualConstraints() {
  if (landmark_container_->NumImages() == 0) { return; }

  // get the keyframe times we want to add constraints to
  std::set<ros::Time> kf_times;
  auto visual_measurements = landmark_container_->GetMeasurementTimes();
  if (params_.init_mode == "VISUAL") {
    // keyframes = poses in init path
    for (const auto& [nsec, pose] : init_path_) {
      const auto timestamp = beam::NSecToRos(nsec);
      if (visual_measurements.find(timestamp) != visual_measurements.end()) {
        kf_times.insert(timestamp);
      }
    }
  } else {
    // sample keyframes manually at 10hz
    ros::Time prev_kf(0.0);
    for (const auto& timestamp : visual_measurements) {
      if ((timestamp - prev_kf).toSec() >= 0.10) {
        kf_times.insert(timestamp);
        prev_kf = timestamp;
      }
    }
  }

  const auto start = beam::NSecToRos(init_path_.begin()->first);
  const auto end = beam::NSecToRos(init_path_.rbegin()->first);
  size_t num_landmarks = 0;
  auto landmark_transaction = fuse_core::Transaction::make_shared();
  landmark_transaction->stamp(start);
  auto process_landmark = [&](const auto& id) {
    // triangulate and add landmark
    const auto initial_point = TriangulateLandmark(id);
    if (!initial_point.has_value()) { return; }

    if (!params_.use_idp) {
      visual_map_->AddLandmark(initial_point.value(), id, landmark_transaction);
      num_landmarks++;

      // add constraints to keyframes that view it
      for (const auto& stamp : kf_times) {
        try {
          Eigen::Vector2d pixel = landmark_container_->GetValue(stamp, id);
          visual_map_->AddVisualConstraint(stamp, id, pixel,
                                           landmark_transaction);
        } catch (const std::out_of_range& oor) { continue; }
      }
    } else {
      // find the first keyframe that has seen the landmark and use it as the
      // anchor
      auto track = landmark_container_->GetTrack(id);
      bool found_anchor = false;
      beam_containers::LandmarkMeasurement anchor_measurement;
      for (auto m : track) {
        if (kf_times.find(m.time_point) != kf_times.end()) {
          anchor_measurement = m;
          found_anchor = true;
          break;
        }
      }
      if (!found_anchor) { return; }

      // get the bearing vector to the measurement
      Eigen::Vector3d bearing;
      Eigen::Vector2i rectified_pixel;
      if (!cam_model_->UndistortPixel(anchor_measurement.value.cast<int>(),
                                      rectified_pixel)) {
        return;
      }
      if (!cam_model_->GetRectifiedModel()->BackProject(rectified_pixel,
                                                        bearing)) {
        return;
      }
      bearing.normalize();

      // find the inverse depth of the point
      auto T_WORLD_CAMERA =
          visual_map_->GetCameraPose(anchor_measurement.time_point);
      if (!T_WORLD_CAMERA.has_value()) { return; }
      Eigen::Vector3d camera_t_point =
          (beam::InvertTransform(T_WORLD_CAMERA.value()) *
           initial_point.value().homogeneous())
              .hnormalized();
      double inverse_depth = 1.0 / camera_t_point.norm();

      visual_map_->AddInverseDepthLandmark(bearing, inverse_depth, id,
                                           anchor_measurement.time_point,
                                           landmark_transaction);
      num_landmarks++;

      // add constraints to keyframes that view it
      for (const auto& stamp : kf_times) {
        try {
          Eigen::Vector2d pixel = landmark_container_->GetValue(stamp, id);
          visual_map_->AddInverseDepthVisualConstraint(stamp, id, pixel,
                                                       landmark_transaction);
        } catch (const std::out_of_range& oor) { continue; }
      }
    }
  };

  // process each landmark
  const auto landmarks =
      landmark_container_->GetLandmarkIDsInWindow(start, end);
  std::for_each(landmarks.begin(), landmarks.end(), process_landmark);

  // send transaction to graph
  local_graph_->update(*landmark_transaction);
  // update visual map with updated graph
  visual_map_->UpdateGraph(*local_graph_);

  ROS_INFO_STREAM(__func__ << ": Added " << num_landmarks
                           << " visual landmarks to initialization graph.");
}

void SLAMInitialization::AddLidarConstraints() {
  if (params_.init_mode != "LIDAR") { return; }

  fuse_core::Transaction::SharedPtr transaction_combined =
      fuse_core::Transaction::make_shared();
  for (const auto& [timeInNs, transaction] :
       lidar_path_init_->GetTransactions()) {
    transaction_combined->merge(*(transaction.GetTransaction()));
  }
  local_graph_->update(*transaction_combined);
}

beam::opt<Eigen::Vector3d>
    SLAMInitialization::TriangulateLandmark(const uint64_t lm_id) {
  std::vector<Eigen::Matrix4d, beam::AlignMat4d> T_cam_world_v;
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  beam_containers::Track track = landmark_container_->GetTrack(lm_id);
  for (auto& m : track) {
    const auto T_camera_world = visual_map_->GetCameraPose(m.time_point);
    // check if the pose is in the graph
    if (T_camera_world.has_value()) {
      pixels.push_back(m.value.cast<int>());
      T_cam_world_v.push_back(beam::InvertTransform(T_camera_world.value()));
    }
  }
  if (T_cam_world_v.size() >= 2) {
    return beam_cv::Triangulation::TriangulatePoint(
        cam_model_, T_cam_world_v, pixels, params_.max_triangulation_distance,
        params_.max_triangulation_reprojection);
  }
  return {};
}

void SLAMInitialization::SendInitializationGraph() {
  const auto first_stamp = beam::NSecToRos(init_path_.begin()->first);
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(first_stamp);
  // add each variable
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
  if (params_.output_folder.empty()) { return; }

  BEAM_INFO("Outputting initialization results to {}", params_.output_folder);

  // create directory if it doesn't exist
  if (!boost::filesystem::exists(params_.output_folder)) {
    boost::filesystem::create_directory(params_.output_folder);
  }

  // create save directory
  std::string save_path = beam::CombinePaths(
      params_.output_folder,
      beam::ConvertTimeToDate(std::chrono::system_clock::now()));
  boost::filesystem::create_directory(save_path);

  // output graph
  std::string graph_txt_path =
      beam::CombinePaths({save_path, "graph_initial.txt"});
  std::ofstream out_graph_file;
  out_graph_file.open(graph_txt_path);
  out_graph_file << graph_before_opt_.rdbuf();
  out_graph_file.close();

  graph_txt_path = beam::CombinePaths({save_path, "graph_optimized.txt"});
  out_graph_file.open(graph_txt_path);
  std::stringstream ss2;
  local_graph_->print(ss2);
  out_graph_file << ss2.rdbuf();
  out_graph_file.close();

  // output trajectory
  auto traj_cloud = bs_common::TrajectoryToCloud(init_path_);
  std::string traj_path =
      beam::CombinePaths({save_path, "initial_trajectory.pcd"});
  std::string error_message;
  if (!beam::SavePointCloud<pcl::PointXYZRGBL>(
          traj_path, traj_cloud, beam::PointCloudFileType::PCDBINARY,
          error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }

  if (lidar_path_init_) {
    std::string lidar_results_path =
        beam::CombinePaths(save_path, "lidar_results");
    boost::filesystem::create_directory(lidar_results_path);
    lidar_path_init_->OutputResults(lidar_results_path);
  }

  std::string graph_poses_init =
      beam::CombinePaths({save_path, "graph_poses_inital.pcd"});
  if (!beam::SavePointCloud<pcl::PointXYZRGBL>(
          graph_poses_init, graph_poses_before_opt_,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }

  std::string graph_poses_opt =
      beam::CombinePaths({save_path, "graph_poses_optimized.pcd"});
  auto graph_poses_after_opt = bs_common::GetGraphPosesAsCloud(*local_graph_);
  if (!beam::SavePointCloud<pcl::PointXYZRGBL>(
          graph_poses_opt, graph_poses_after_opt,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }

  std::string imu_biases_plot =
      beam::CombinePaths({save_path, "imu_biases_optimized.pdf"});
  bs_common::PlotImuBiasesFromGraph(*local_graph_, imu_biases_plot);

  // draw graph
  auto imu_constraints =
      graph_visualization::GetGraphRelativeImuConstraintsAsCloud(*local_graph_,
                                                                 0.01, 0.15);
  auto lidar_constraints = bs_common::GetGraphRelativePoseConstraintsAsCloud(
      *local_graph_, "LidarOdometry::");
  if (lidar_constraints.empty()) {
    lidar_constraints = graph_visualization::
        GetGraphRelativePoseWithExtrinsicsConstraintsAsCloud(*local_graph_,
                                                             "LidarOdometry::");
  }
  std::string imu_constraints_path =
      beam::CombinePaths({save_path, "imu_constraints_initial.pcd"});
  if (!beam::SavePointCloud<pcl::PointXYZRGBL>(
          imu_constraints_path, imu_constraints_before_opt_,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }

  imu_constraints_path =
      beam::CombinePaths({save_path, "imu_constraints_opt.pcd"});
  if (!beam::SavePointCloud<pcl::PointXYZRGBL>(
          imu_constraints_path, imu_constraints,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }

  std::string lidar_constraints_path =
      beam::CombinePaths({save_path, "lidar_constraints_opt.pcd"});
  if (!beam::SavePointCloud<pcl::PointXYZRGBL>(
          lidar_constraints_path, lidar_constraints,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }

  lidar_constraints_path =
      beam::CombinePaths({save_path, "lidar_constraints_initial.pcd"});
  if (!beam::SavePointCloud<pcl::PointXYZRGBL>(
          lidar_constraints_path, lidar_constraints_before_opt_,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }
}

void SLAMInitialization::shutdown() {
  visual_measurement_subscriber_.shutdown();
  imu_subscriber_.shutdown();
  lidar_subscriber_.shutdown();
  imu_buffer_.clear();
  frame_init_buffer_.clear();
  local_graph_->clear();
  frame_initializer_ = nullptr;
  visual_map_ = nullptr;
  imu_preint_ = nullptr;
  lidar_path_init_ = nullptr;
}

void SLAMInitialization::AddMeasurementsToContainer(
    const bs_common::CameraMeasurementMsg::ConstPtr& msg) {
  // check that message hasnt already been added to container
  const auto times = landmark_container_->GetMeasurementTimes();
  if (times.find(msg->header.stamp) != times.end()) { return; }

  std::map<uint64_t, Eigen::Vector2d> cur_undistorted_measurements;

  // put all measurements into landmark container
  for (const auto& lm : msg->landmarks) {
    Eigen::Vector2d landmark(static_cast<double>(lm.pixel_u),
                             static_cast<double>(lm.pixel_v));
    const cv::Mat landmark_descriptor =
        beam_cv::Descriptor::VectorDescriptorToCvMat({lm.descriptor.data},
                                                     msg->descriptor_type);
    beam_containers::LandmarkMeasurement lm_measurement(
        msg->header.stamp, msg->sensor_id, lm.landmark_id, msg->header.seq,
        landmark, landmark_descriptor);
    landmark_container_->Insert(lm_measurement);

    Eigen::Vector2i rectified_pixel;
    if (cam_model_->UndistortPixel(landmark.cast<int>(), rectified_pixel)) {
      cur_undistorted_measurements.insert(
          {lm.landmark_id, rectified_pixel.cast<double>()});
    }
  }

  // get previous frame undistorted measurements
  if (prev_frame_ != ros::Time(0)) {
    std::map<uint64_t, Eigen::Vector2d> prev_undistorted_measurements;
    std::vector<uint64_t> landmarks =
        landmark_container_->GetLandmarkIDsInImage(prev_frame_);
    for (auto& id : landmarks) {
      try {
        const Eigen::Vector2d prev_measurement =
            landmark_container_->GetValue(prev_frame_, id);

        Eigen::Vector2i rectified_pixel;
        if (cam_model_->UndistortPixel(prev_measurement.cast<int>(),
                                       rectified_pixel)) {
          prev_undistorted_measurements.insert(
              {id, rectified_pixel.cast<double>()});
        }
      } catch (const std::out_of_range& oor) {}
    }

    // get matches to previous frame
    std::vector<cv::Point2f> fp1, fp2;
    std::vector<uint64_t> matched_ids;
    for (const auto& [id, pixel] : prev_undistorted_measurements) {
      if (cur_undistorted_measurements.find(id) !=
          cur_undistorted_measurements.end()) {
        cv::Point2f p1 =
            beam_cv::ConvertKeypoint(prev_undistorted_measurements[id]);
        cv::Point2f p2 =
            beam_cv::ConvertKeypoint(cur_undistorted_measurements[id]);
        fp1.push_back(p1);
        fp2.push_back(p2);
        matched_ids.push_back(id);
      }
    }

    // attempt essential matrix estimation
    std::vector<uchar> mask;
    cv::findEssentialMat(fp1, fp2, K_, cv::RANSAC, 0.99,
                         params_.track_outlier_pixel_threshold, mask);

    // remove outliers from container
    for (size_t i = 0; i < mask.size(); i++) {
      if (mask.at(i) == 0) {
        const auto id = matched_ids[i];
        landmark_container_->Erase(msg->header.stamp, id);
      }
    }
  }
  prev_frame_ = msg->header.stamp;
}

} // namespace bs_models
