#include <beam_models/initialization/vio_initializer.h>

namespace beam_models { namespace camera_to_camera {

VIOInitializer::VIOInitializer(
    std::shared_ptr<beam_calibration::CameraModel> cam_model,
    std::shared_ptr<beam_cv::Tracker> tracker,
    std::shared_ptr<beam_cv::PoseRefinement> pose_refiner,
    const Eigen::Matrix4d& T_imu_cam, bool use_scale_estimate)
    : cam_model_(cam_model),
      tracker_(tracker),
      pose_refiner_(pose_refiner),
      T_imu_cam_(T_imu_cam),
      use_scale_estimate_(use_scale_estimate) {
  local_graph_ = std::make_shared<fuse_graphs::HashGraph>();
  visual_map_ =
      std::make_shared<VisualMap>(cam_model_, local_graph_, T_imu_cam);
}

bool VIOInitializer::AddImage(ros::Time cur_time) {
  frame_times_.push_back(cur_time.toNSec());
  if (init_path_) {
    start_ = init_path_->poses[0].header.stamp;
    end_ = init_path_->poses[init_path_->poses.size() - 1].header.stamp;
    // Build frame vectors (valid frames are those within [start_, end_])
    std::vector<beam_models::camera_to_camera::Frame> valid_frames;
    std::vector<beam_models::camera_to_camera::Frame> invalid_frames;
    BuildFrameVectors(valid_frames, invalid_frames);
    // Initialize imu preintegration
    PerformIMUInitialization(valid_frames);
    beam_models::frame_to_frame::ImuPreintegration::Params imu_params;
    imu_params.gravity = gravity_;
    imu_preint_ =
        std::make_shared<beam_models::frame_to_frame::ImuPreintegration>(
            imu_params, bg_, ba_);
    // Apply scale estimate if desired
    if (use_scale_estimate_)
      for (auto& f : valid_frames) { f.p = scale_ * f.p; }
    // Add poses and imu constraints to graph
    AddPosesAndIMUConstraints(valid_frames);
    // Add landmarks and visual constraints to graph
    AddLandmarksAndVisualConstraints();
    // optimize graph
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    local_graph_->optimize(options);
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
  valid_frames.clear();
  invalid_frames.clear();
  // get start and end time of input path
  for (auto& kf : frame_times_) {
    ros::Time stamp;
    stamp.fromNSec(kf);
    if (stamp < start_ || stamp > end_) continue;
    // add imu data to frames preintegrator
    beam_common::PreIntegrator preintegrator;
    while (imu_buffer_.front().header.stamp <= stamp) {
      beam_common::IMUData imu_data(imu_buffer_.front());
      preintegrator.data.push_back(imu_data);
      imu_buffer_.pop();
    }
    // if (stamp < end) {
    // get pose of frame using path
    Eigen::Matrix4d T_WORLD_IMU;
    beam_common::InterpolateTransformFromPath(init_path_->poses, stamp, T_WORLD_IMU);
    Eigen::Vector3d p_WORLD_IMU;
    Eigen::Quaterniond q_WORLD_IMU;
    beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_IMU, q_WORLD_IMU,
                                                    p_WORLD_IMU);
    // create frame and add to valid frame vector
    beam_models::camera_to_camera::Frame new_frame(stamp, p_WORLD_IMU,
                                                   q_WORLD_IMU, preintegrator);
    valid_frames.push_back(new_frame);
    //}
    // else {
    //   // get pose of frame using path
    //   Eigen::Matrix4d T_WORLD_IMU = Eigen::Matrix4d::Zero();
    //   Eigen::Vector3d p_WORLD_IMU;
    //   Eigen::Quaterniond q_WORLD_IMU;
    //   beam::TransformMatrixToQuaternionAndTranslation(
    //       T_WORLD_IMU, q_WORLD_IMU, p_WORLD_IMU);
    //   // create frame and add to invalid frame vector
    //   beam_models::camera_to_camera::Frame new_frame(
    //       stamp, p_WORLD_IMU, q_WORLD_IMU, preintegrator);
    //   invalid_frames.push_back(new_frame);
    // }
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

void VIOInitializer::AddPosesAndIMUConstraints(
    const std::vector<beam_models::camera_to_camera::Frame>& frames) {
  // add initial poses and imu constraints
  for (int i = 0; i < frames.size(); i++) {
    // 1. Add frames pose to graph
    beam_models::camera_to_camera::Frame frame = frames[i];
    Eigen::Matrix4d T_WORLD_IMU;
    beam::QuaternionAndTranslationToTransformMatrix(frame.q, frame.p,
                                                    T_WORLD_IMU);
    Eigen::Matrix4d T_WORLD_CAM = T_WORLD_IMU * T_imu_cam_;
    visual_map_->AddPose(T_WORLD_CAM, frame.t);
    // 2. Push its imu messages
    for (auto& imu_data : frame.preint.data) {
      imu_preint_->AddToBuffer(imu_data);
    }
    // 3. Add respective imu constraints
    if (i == 0) {
      imu_preint_->SetStart(frame.t, visual_map_->GetOrientation(frame.t),
                            visual_map_->GetPosition(frame.t));
    } else {
      fuse_variables::Orientation3DStamped::SharedPtr img_orientation =
          visual_map_->GetOrientation(frame.t);
      fuse_variables::Position3DStamped::SharedPtr img_position =
          visual_map_->GetPosition(frame.t);
      auto transaction = imu_preint_
                             ->RegisterNewImuPreintegratedFactor(
                                 frame.t, img_orientation, img_position)
                             .GetTransaction();
      for (auto& var : transaction->addedVariables()) {
        fuse_core::Variable::UniquePtr var_unique = var.clone();
        fuse_core::Variable::SharedPtr var_shared = std::move(var_unique);
        local_graph_->addVariable(var_shared);
      }
      for (auto& constraint : transaction->addedConstraints()) {
        fuse_core::Constraint::UniquePtr constraint_unique = constraint.clone();
        fuse_core::Constraint::SharedPtr constraint_shared =
            std::move(constraint_unique);
        local_graph_->addConstraint(constraint_shared);
      }
    }
  }
}

void VIOInitializer::AddLandmarksAndVisualConstraints() {
  std::vector<uint64_t> landmarks =
      tracker_->GetLandmarkIDsInWindow(start_, end_);
  int num_landmarks = 0;
  for (auto& id : landmarks) {
    if (!visual_map_->GetLandmark(id)) {
      std::vector<Eigen::Matrix4d, beam_cv::AlignMat4d> T_cam_world_v;
      std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
      std::vector<ros::Time> observation_stamps;
      beam_cv::FeatureTrack track = tracker_->GetTrack(id);
      for (auto& m : track) {
        beam::opt<Eigen::Matrix4d> T = visual_map_->GetPose(m.time_point);
        // check if the pose is in the graph (keyframe)
        if (T.has_value()) {
          pixels.push_back(m.value.cast<int>());
          T_cam_world_v.push_back(T.value());
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
  std::cout << "Initial map landmarks: " << num_landmarks << std::endl;
}

}} // namespace beam_models::camera_to_camera