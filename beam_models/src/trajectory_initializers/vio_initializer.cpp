#include <beam_models/trajectory_initializers/vio_initializer.h>

#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/PoseRefinement.h>

namespace beam_models { namespace camera_to_camera {

VIOInitializer::VIOInitializer(
    std::shared_ptr<beam_calibration::CameraModel> cam_model,
    std::shared_ptr<beam_cv::Tracker> tracker, const double& gyro_noise,
    const double& accel_noise, const double& gyro_bias,
    const double& accel_bias, bool use_scale_estimate)
    : cam_model_(cam_model),
      tracker_(tracker),
      use_scale_estimate_(use_scale_estimate) {
  // set covariance matrices for imu
  cov_gyro_noise_ = Eigen::Matrix3d::Identity() * gyro_noise;
  cov_accel_noise_ = Eigen::Matrix3d::Identity() * accel_noise;
  cov_gyro_bias_ = Eigen::Matrix3d::Identity() * gyro_bias;
  cov_accel_bias_ = Eigen::Matrix3d::Identity() * accel_bias;
  // create optimzation graph
  local_graph_ = std::make_shared<fuse_graphs::HashGraph>();
  // create visual map
  visual_map_ = std::make_shared<VisualMap>(cam_model_, local_graph_);
}

bool VIOInitializer::AddImage(ros::Time cur_time) {
  frame_times_.push_back(cur_time.toNSec());
  if (init_path_) {
    ROS_INFO("Attempting Initialization.");
    // Build frame vectors
    std::vector<beam_models::camera_to_camera::Frame> valid_frames;
    std::vector<beam_models::camera_to_camera::Frame> invalid_frames;
    BuildFrameVectors(valid_frames, invalid_frames);
    // Initialize imu preintegration
    PerformIMUInitialization(valid_frames);
    beam_models::frame_to_frame::ImuPreintegration::Params imu_params;
    imu_params.gravity = gravity_;
    imu_params.cov_gyro_noise = cov_gyro_noise_;
    imu_params.cov_accel_noise = cov_accel_noise_;
    imu_params.cov_gyro_bias = cov_gyro_bias_;
    imu_params.cov_accel_bias = cov_accel_bias_;
    imu_preint_ =
        std::make_shared<beam_models::frame_to_frame::ImuPreintegration>(
            imu_params, bg_, ba_);
    // Apply scale estimate if desired
    if (use_scale_estimate_)
      for (auto& f : valid_frames) { f.p = scale_ * f.p; }
    // Add poses from path and imu constraints to graph
    AddPosesAndInertialConstraints(valid_frames, true);
    // Add landmarks and visual constraints to graph
    size_t init_lms = AddVisualConstraints(valid_frames);
    // optimize graph
    OptimizeGraph();
    // localize the frames that are outside of the given path
    for (auto& f : invalid_frames) {
      Eigen::Matrix4d T_WORLD_CAMERA;
      LocalizeFrame(f, T_WORLD_CAMERA);
      beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_CAMERA, f.q, f.p);
    }
    // add localized poses and imu constraints
    AddPosesAndInertialConstraints(invalid_frames, false);
    // add landmarks and visual constraints for the invalid frames
    init_lms += AddVisualConstraints(invalid_frames);
    // for (auto& f : valid_frames) {
    //   std::cout << f.t << std::endl;
    //   std::cout << visual_map_->GetPose(f.t) << std::endl;
    // }
    // for (auto& f : invalid_frames) {
    //   std::cout << f.t << std::endl;
    //   std::cout << visual_map_->GetPose(f.t) << std::endl;
    // }
    ROS_INFO("Initialized Map Points: %zu", init_lms);
    is_initialized_ = true;
  }
  return is_initialized_;
}

void VIOInitializer::AddIMU(const sensor_msgs::Imu& msg) {
  imu_buffer_.push(msg);
}

void VIOInitializer::SetPath(const InitializedPathMsg& msg) {
  init_path_ = std::make_shared<InitializedPathMsg>();
  *init_path_ = msg;
}

bool VIOInitializer::Initialized() {
  return is_initialized_;
}

const fuse_graphs::HashGraph& VIOInitializer::GetGraph() {
  return *local_graph_;
}

std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration>
    VIOInitializer::GetPreintegrator() {
  return imu_preint_;
}

void VIOInitializer::BuildFrameVectors(
    std::vector<beam_models::camera_to_camera::Frame>& valid_frames,
    std::vector<beam_models::camera_to_camera::Frame>& invalid_frames) {
  ros::Time start = init_path_->poses[0].header.stamp;
  ros::Time end = init_path_->poses[init_path_->poses.size() - 1].header.stamp;
  valid_frames.clear();
  invalid_frames.clear();
  // get start and end time of input path
  for (auto& kf : frame_times_) {
    ros::Time stamp;
    stamp.fromNSec(kf);
    if (stamp < start) continue;
    // add imu data to frames preintegrator
    beam_common::PreIntegrator preintegrator;
    preintegrator.cov_w = cov_gyro_noise_;
    preintegrator.cov_a = cov_accel_noise_;
    preintegrator.cov_bg = cov_gyro_bias_;
    preintegrator.cov_ba = cov_accel_bias_;
    while (imu_buffer_.front().header.stamp <= stamp && !imu_buffer_.empty()) {
      beam_common::IMUData imu_data(imu_buffer_.front());
      preintegrator.data.push_back(imu_data);
      imu_buffer_.pop();
    }
    if (stamp <= end) {
      // get pose of frame using path
      Eigen::Matrix4d T_WORLD_IMU;
      beam_common::InterpolateTransformFromPath(init_path_->poses, stamp,
                                                T_WORLD_IMU);
      Eigen::Vector3d p_WORLD_IMU;
      Eigen::Quaterniond q_WORLD_IMU;
      beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_IMU, q_WORLD_IMU,
                                                      p_WORLD_IMU);
      // create frame and add to valid frame vector
      beam_models::camera_to_camera::Frame new_frame{
          stamp, p_WORLD_IMU, q_WORLD_IMU, preintegrator};
      valid_frames.push_back(new_frame);
    } else {
      // get arbitrary pose values
      Eigen::Vector3d p_WORLD_IMU(0, 0, 0);
      Eigen::Quaterniond q_WORLD_IMU(0, 0, 0, 0);
      // create frame and add to invalid frame vector
      beam_models::camera_to_camera::Frame new_frame{
          stamp, p_WORLD_IMU, q_WORLD_IMU, preintegrator};
      invalid_frames.push_back(new_frame);
    }
  }
}

void VIOInitializer::PerformIMUInitialization(
    const std::vector<beam_models::camera_to_camera::Frame>& frames) {
  beam_models::camera_to_camera::IMUInitializer imu_init(frames);
  if (init_path_->gyroscope_bias.x == 0 && init_path_->gyroscope_bias.y == 0 &&
      init_path_->gyroscope_bias.z == 0) {
    imu_init.SolveGyroBias();
    bg_ = imu_init.GetGyroBias();
  } else {
    bg_ << init_path_->gyroscope_bias.x, init_path_->gyroscope_bias.y,
        init_path_->gyroscope_bias.z;
  }
  if (init_path_->gravity.x == 0 && init_path_->gravity.y == 0 &&
      init_path_->gravity.z == 0) {
    imu_init.SolveGravityAndScale();
    imu_init.RefineGravityAndScale();
    gravity_ = imu_init.GetGravity();
    scale_ = imu_init.GetScale();
  } else {
    gravity_ << init_path_->gravity.x, init_path_->gravity.y,
        init_path_->gravity.z;
    scale_ = init_path_->scale;
  }
  if (init_path_->accelerometer_bias.x == 0 &&
      init_path_->accelerometer_bias.y == 0 &&
      init_path_->accelerometer_bias.z == 0) {
    imu_init.SolveAccelBias();
    ba_ = imu_init.GetAccelBias();
  } else {
    ba_ << init_path_->accelerometer_bias.x, init_path_->accelerometer_bias.y,
        init_path_->accelerometer_bias.z;
  }
}

void VIOInitializer::AddPosesAndInertialConstraints(
    const std::vector<beam_models::camera_to_camera::Frame>& frames,
    bool set_start) {
  // add initial poses and imu data to preintegrator
  for (int i = 0; i < frames.size(); i++) {
    // 1. Add frame's pose to graph
    beam_models::camera_to_camera::Frame frame = frames[i];
    visual_map_->AddPosition(frame.p, frame.t);
    visual_map_->AddOrientation(frame.q, frame.t);
    // 2. Push its imu messages
    for (auto& imu_data : frame.preint.data) {
      imu_preint_->AddToBuffer(imu_data);
    }
  }
  // add inertial constraints between each frame
  for (int i = 0; i < frames.size(); i++) {
    beam_models::camera_to_camera::Frame frame = frames[i];
    fuse_variables::Orientation3DStamped::SharedPtr img_orientation =
        visual_map_->GetOrientation(frame.t);
    fuse_variables::Position3DStamped::SharedPtr img_position =
        visual_map_->GetPosition(frame.t);
    // 3. Add respective imu constraints
    if (set_start && i == 0) {
      imu_preint_->SetStart(frame.t, img_orientation, img_position);
    } else {
      // get imu transaction
      fuse_core::Transaction::SharedPtr transaction =
          imu_preint_->RegisterNewImuPreintegratedFactor(
              frame.t, img_orientation, img_position);
      // add constituent variables and constraints
      for (auto& var : transaction->addedVariables()) {
        local_graph_->addVariable(std::move(var.clone()));
      }
      for (auto& constraint : transaction->addedConstraints()) {
        local_graph_->addConstraint(std::move(constraint.clone()));
      }
    }
  }
}

size_t VIOInitializer::AddVisualConstraints(
    const std::vector<beam_models::camera_to_camera::Frame>& frames) {
  ros::Time start = frames[0].t, end = frames[frames.size() - 1].t;
  size_t num_landmarks = 0;

  std::vector<uint64_t> landmarks =
      tracker_->GetLandmarkIDsInWindow(start, end);
  for (auto& id : landmarks) {
    fuse_variables::Position3D::SharedPtr lm = visual_map_->GetLandmark(id);
    if (lm) {
      for (auto& f : frames) {
        try {
          visual_map_->AddConstraint(f.t, id, tracker_->Get(f.t, id));
        } catch (const std::out_of_range& oor) {}
      }
    } else {
      std::vector<Eigen::Matrix4d, beam_cv::AlignMat4d> T_cam_world_v;
      std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
      std::vector<ros::Time> observation_stamps;
      beam_cv::FeatureTrack track = tracker_->GetTrack(id);
      for (auto& m : track) {
        beam::opt<Eigen::Matrix4d> T = visual_map_->GetPose(m.time_point);
        // check if the pose is in the graph (keyframe)
        if (T.has_value()) {
          pixels.push_back(m.value.cast<int>());
          T_cam_world_v.push_back(T.value().inverse());
          observation_stamps.push_back(m.time_point);
        }
      }
      if (T_cam_world_v.size() >= 2) {
        beam::opt<Eigen::Vector3d> point =
            beam_cv::Triangulation::TriangulatePoint(cam_model_, T_cam_world_v,
                                                     pixels);
        if (point.has_value()) {
          num_landmarks++;
          visual_map_->AddLandmark(point.value(), id);
          for (int i = 0; i < observation_stamps.size(); i++) {
            visual_map_->AddConstraint(
                observation_stamps[i], id,
                tracker_->Get(observation_stamps[i], id));
          }
        }
      }
    }
  }

  return num_landmarks;
}

bool VIOInitializer::LocalizeFrame(
    const beam_models::camera_to_camera::Frame& frame,
    Eigen::Matrix4d& T_WORLD_CAMERA) {
  std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam_cv::AlignVec3d> points;
  std::vector<uint64_t> landmarks = tracker_->GetLandmarkIDsInImage(frame.t);
  for (auto& id : landmarks) {
    fuse_variables::Position3D::SharedPtr lm = visual_map_->GetLandmark(id);
    if (lm) {
      Eigen::Vector2i pixeli = tracker_->Get(frame.t, id).cast<int>();
      pixels.push_back(pixeli);
      Eigen::Vector3d point(lm->x(), lm->y(), lm->z());
      points.push_back(point);
    }
  }
  if (points.size() < 3) { return false; }
  Eigen::Matrix4d T_CAMERA_WORLD_est =
      beam_cv::AbsolutePoseEstimator::RANSACEstimator(cam_model_, pixels,
                                                      points);
  ceres::Solver::Options ceres_solver_options;
  ceres_solver_options.minimizer_progress_to_stdout = false;
  ceres_solver_options.max_solver_time_in_seconds = 1e-1;
  ceres_solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  ceres_solver_options.preconditioner_type = ceres::SCHUR_JACOBI;
  beam_cv::PoseRefinement refiner(ceres_solver_options);
  std::string report;
  Eigen::Matrix4d T_CAMERA_WORLD_ref = refiner.RefinePose(
      T_CAMERA_WORLD_est, cam_model_, pixels, points, report);
  T_WORLD_CAMERA = T_CAMERA_WORLD_ref.inverse();
  return true;
}

void VIOInitializer::OptimizeGraph() {
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_solver_time_in_seconds = 5;
  local_graph_->optimize(options);
}

}} // namespace beam_models::camera_to_camera