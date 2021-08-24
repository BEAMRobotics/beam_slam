#include <bs_models/trajectory_initializers/vio_initializer.h>

#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/PoseRefinement.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_utils/utils.h>
#include <bs_common/utils.h>

#include <boost/filesystem.hpp>

namespace bs_models { namespace camera_to_camera {

VIOInitializer::VIOInitializer(
    std::shared_ptr<beam_calibration::CameraModel> cam_model,
    std::shared_ptr<beam_cv::Tracker> tracker, const double& gyro_noise,
    const double& accel_noise, const double& gyro_bias,
    const double& accel_bias, bool use_scale_estimate,
    double max_optimization_time, const std::string& output_directory)
    : cam_model_(cam_model),
      tracker_(tracker),
      use_scale_estimate_(use_scale_estimate),
      max_optimization_time_(max_optimization_time),
      output_directory_(output_directory) {
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
    ROS_INFO("Attempting VIO Initialization.");
    // Build frame vectors
    std::vector<bs_models::camera_to_camera::Frame> valid_frames;
    std::vector<bs_models::camera_to_camera::Frame> invalid_frames;
    BuildFrameVectors(valid_frames, invalid_frames);
    // Initialize imu preintegration
    PerformIMUInitialization(valid_frames);
    Eigen::Vector3d gravity_nominal{0, 0, -9.8};
    bs_models::frame_to_frame::ImuPreintegration::Params imu_params;
    imu_params.gravity = gravity_nominal;
    imu_params.cov_gyro_noise = cov_gyro_noise_;
    imu_params.cov_accel_noise = cov_accel_noise_;
    imu_params.cov_gyro_bias = cov_gyro_bias_;
    imu_params.cov_accel_bias = cov_accel_bias_;
    imu_preint_ =
        std::make_shared<bs_models::frame_to_frame::ImuPreintegration>(
            imu_params);
    // align poses to estimated gravity
    Eigen::Quaterniond q =
        Eigen::Quaterniond::FromTwoVectors(gravity_, gravity_nominal);
    for (auto& f : valid_frames) {
      f.q = q * f.q;
      f.p = q * f.p;
      // Apply scale estimate if desired
      if (use_scale_estimate_) f.p = scale_ * f.p;
    }
    // Add poses from path and imu constraints to graph
    AddPosesAndInertialConstraints(valid_frames, true);

    // TODO: refine biases through optimization awith constant poses

    // Add landmarks and visual constraints to graph
    size_t init_lms = AddVisualConstraints(valid_frames);
    // optimize valid frames
    std::cout << "\n\nFrame poses before optimization:\n" << std::endl;
    OutputFramePoses(valid_frames);
    OptimizeGraph();
    std::cout << "\n\nFrame poses after optimization:\n" << std::endl;
    OutputFramePoses(valid_frames);
    // localize the frames that are outside of the given path
    for (auto& f : invalid_frames) {
      Eigen::Matrix4d T_WORLD_CAMERA;
      // if failure to localize next frame then init is a failure
      if (!LocalizeFrame(f, T_WORLD_CAMERA)) { return false; }
      beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_CAMERA, f.q, f.p);
    }
    // add localized poses and imu constraints
    AddPosesAndInertialConstraints(invalid_frames, false);
    // add landmarks and visual constraints for the invalid frames
    init_lms += AddVisualConstraints(invalid_frames);
    // log initialization statistics
    ROS_INFO("Initialized Map Points: %zu", init_lms);
    is_initialized_ = true;
  }
  return is_initialized_;
}

void VIOInitializer::OutputFramePoses(
    const std::vector<bs_models::camera_to_camera::Frame>& frames) {
  for (auto& f : frames) {
    std::cout << f.t << std::endl;
    std::cout << visual_map_->GetCameraPose(f.t) << std::endl;
  }
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

std::shared_ptr<bs_models::frame_to_frame::ImuPreintegration>
    VIOInitializer::GetPreintegrator() {
  return imu_preint_;
}

void VIOInitializer::BuildFrameVectors(
    std::vector<bs_models::camera_to_camera::Frame>& valid_frames,
    std::vector<bs_models::camera_to_camera::Frame>& invalid_frames) {
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
    bs_common::PreIntegrator preintegrator;
    preintegrator.cov_w = cov_gyro_noise_;
    preintegrator.cov_a = cov_accel_noise_;
    preintegrator.cov_bg = cov_gyro_bias_;
    preintegrator.cov_ba = cov_accel_bias_;
    while (imu_buffer_.front().header.stamp <= stamp && !imu_buffer_.empty()) {
      bs_common::IMUData imu_data(imu_buffer_.front());
      preintegrator.data.push_back(imu_data);
      imu_buffer_.pop();
    }
    if (stamp <= end) {
      // get pose of frame using path
      Eigen::Matrix4d T_WORLD_BASELINK;
      bs_common::InterpolateTransformFromPath(init_path_->poses, stamp,
                                              T_WORLD_BASELINK);
      Eigen::Vector3d p_WORLD_BASELINK;
      Eigen::Quaterniond q_WORLD_BASELINK;
      beam::TransformMatrixToQuaternionAndTranslation(
          T_WORLD_BASELINK, q_WORLD_BASELINK, p_WORLD_BASELINK);
      // create frame and add to valid frame vector
      bs_models::camera_to_camera::Frame new_frame{
          stamp, p_WORLD_BASELINK, q_WORLD_BASELINK, preintegrator};
      valid_frames.push_back(new_frame);
    } else {
      // get arbitrary pose values
      Eigen::Vector3d p_WORLD_BASELINK(0, 0, 0);
      Eigen::Quaterniond q_WORLD_BASELINK(0, 0, 0, 0);
      // create frame and add to invalid frame vector
      bs_models::camera_to_camera::Frame new_frame{
          stamp, p_WORLD_BASELINK, q_WORLD_BASELINK, preintegrator};
      invalid_frames.push_back(new_frame);
    }
  }
}

void VIOInitializer::PerformIMUInitialization(
    const std::vector<bs_models::camera_to_camera::Frame>& frames) {
  bs_models::camera_to_camera::IMUInitializer imu_init(frames);
  // estimate gyroscope bias
  if (init_path_->gyroscope_bias.x == 0 && init_path_->gyroscope_bias.y == 0 &&
      init_path_->gyroscope_bias.z == 0) {
    imu_init.SolveGyroBias();
    bg_ = imu_init.GetGyroBias();
  } else {
    bg_ << init_path_->gyroscope_bias.x, init_path_->gyroscope_bias.y,
        init_path_->gyroscope_bias.z;
  }
  // estimate gravity and scale
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
  // estimate accelerometer bias
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
    const std::vector<bs_models::camera_to_camera::Frame>& frames,
    bool set_start) {
  // add initial poses and imu data to preintegrator
  for (int i = 0; i < frames.size(); i++) {
    // Add frame's pose to graph
    bs_models::camera_to_camera::Frame frame = frames[i];
    visual_map_->AddPosition(frame.p, frame.t);
    visual_map_->AddOrientation(frame.q, frame.t);
    Eigen::Matrix4d T;
    beam::QuaternionAndTranslationToTransformMatrix(frame.q, frame.p, T);
    // Push its imu messages
    for (auto& imu_data : frame.preint.data) {
      imu_preint_->AddToBuffer(imu_data);
    }

    fuse_variables::Orientation3DStamped::SharedPtr img_orientation =
        visual_map_->GetOrientation(frame.t);
    fuse_variables::Position3DStamped::SharedPtr img_position =
        visual_map_->GetPosition(frame.t);
    // Add respective imu constraints
    if (set_start && i == 0) {
      imu_preint_->SetStart(frame.t, img_orientation, img_position);
    } else {
      // get imu transaction
      fuse_core::Transaction::SharedPtr transaction =
          imu_preint_->RegisterNewImuPreintegratedFactor(
              frame.t, img_orientation, img_position);
      // update graph with the transaction
      // local_graph_->update(*transaction);
    }
  }
}

size_t VIOInitializer::AddVisualConstraints(
    const std::vector<bs_models::camera_to_camera::Frame>& frames) {
  ros::Time start = frames[0].t, end = frames[frames.size() - 1].t;
  size_t num_landmarks = 0;
  // get all landmarks in the window
  std::vector<uint64_t> landmarks =
      tracker_->GetLandmarkIDsInWindow(start, end);
  for (auto& id : landmarks) {
    fuse_variables::Point3DLandmark::SharedPtr lm =
        visual_map_->GetLandmark(id);
    if (lm) { // if the landmark already exists then add constraint
      for (auto& f : frames) {
        try {
          visual_map_->AddConstraint(f.t, id, tracker_->Get(f.t, id));
        } catch (const std::out_of_range& oor) {}
      }
    } else { // otherwise then triangulate then add the constraints
      std::vector<Eigen::Matrix4d, beam_cv::AlignMat4d> T_cam_world_v;
      std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
      std::vector<ros::Time> observation_stamps;
      beam_cv::FeatureTrack track = tracker_->GetTrack(id);
      for (auto& m : track) {
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
    const bs_models::camera_to_camera::Frame& frame,
    Eigen::Matrix4d& T_WORLD_BASELINK) {
  std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam_cv::AlignVec3d> points;
  std::vector<uint64_t> landmarks = tracker_->GetLandmarkIDsInImage(frame.t);
  // get 2d-3d correspondences
  for (auto& id : landmarks) {
    fuse_variables::Point3DLandmark::SharedPtr lm =
        visual_map_->GetLandmark(id);
    if (lm) {
      Eigen::Vector2i pixeli = tracker_->Get(frame.t, id).cast<int>();
      pixels.push_back(pixeli);
      Eigen::Vector3d point(lm->x(), lm->y(), lm->z());
      points.push_back(point);
    }
  }
  if (points.size() < 15) { return false; }
  // estimate with ransac pnp
  Eigen::Matrix4d T_CAMERA_WORLD_est =
      beam_cv::AbsolutePoseEstimator::RANSACEstimator(cam_model_, pixels,
                                                      points);
  ceres::Solver::Options ceres_solver_options;
  ceres_solver_options.minimizer_progress_to_stdout = false;
  ceres_solver_options.max_solver_time_in_seconds = 1e-1;
  ceres_solver_options.logging_type = ceres::SILENT;
  ceres_solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  ceres_solver_options.preconditioner_type = ceres::SCHUR_JACOBI;
  beam_cv::PoseRefinement refiner(ceres_solver_options);
  // refine pose using reprojection error
  Eigen::Matrix4d T_CAMERA_WORLD_ref =
      refiner.RefinePose(T_CAMERA_WORLD_est, cam_model_, pixels, points);
  Eigen::Matrix4d T_WORLD_CAMERA = T_CAMERA_WORLD_ref.inverse();
  extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_);
  // transform into baselink frame
  T_WORLD_BASELINK = T_WORLD_CAMERA * T_cam_baselink_;
  return true;
}

void VIOInitializer::OptimizeGraph() {
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 6;
  options.num_linear_solver_threads = 6;
  options.minimizer_type = ceres::TRUST_REGION;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.max_solver_time_in_seconds = max_optimization_time_;
  options.max_num_iterations = 100;
  local_graph_->optimize(options);
}

void VIOInitializer::OutputResults(
    const std::vector<bs_models::camera_to_camera::Frame>& frames) {
  if (!boost::filesystem::exists(output_directory_)) {
    if (!output_directory_.empty()) {
      ROS_ERROR("Output directory does not exist. Not outputting VIO "
                "Initializer results.");
    }
  } else {
    // add frame poses to cloud and save
    pcl::PointCloud<pcl::PointXYZRGB> frame_cloud;
    for (auto& f : frames) {
      frame_cloud = beam::AddFrameToCloud(
          frame_cloud, visual_map_->GetCameraPose(f.t).value(), 0.001);
    }
    // add all landmark points to cloud and save
    std::vector<uint64_t> landmarks = tracker_->GetLandmarkIDsInWindow(
        frames[0].t, frames[frames.size() - 1].t);
    pcl::PointCloud<pcl::PointXYZ> points_cloud;
    for (auto& id : landmarks) {
      fuse_variables::Point3DLandmark::SharedPtr lm =
          visual_map_->GetLandmark(id);
      if (lm) {
        pcl::PointXYZ p(lm->x(), lm->y(), lm->z());
        points_cloud.points.push_back(p);
      }
    }
    pcl::io::savePCDFileBinary(output_directory_ + "/frames.pcd", frame_cloud);
    pcl::io::savePCDFileBinary(output_directory_ + "/points.pcd", points_cloud);
  }
}

}} // namespace bs_models::camera_to_camera