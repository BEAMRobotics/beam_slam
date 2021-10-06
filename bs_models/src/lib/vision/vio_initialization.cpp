#include <bs_models/vision/vio_initialization.h>

#include <nlohmann/json.hpp>

#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_utils/filesystem.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

#include <bs_common/utils.h>

namespace bs_models {
namespace vision {

VIOInitialization::VIOInitialization(
    std::shared_ptr<beam_calibration::CameraModel> cam_model,
    std::shared_ptr<beam_cv::Tracker> tracker, const std::string &path_topic,
    const std::string &imu_intrinsics_path, bool use_scale_estimate,
    double max_optimization_time, const std::string &output_directory)
    : cam_model_(cam_model), tracker_(tracker),
      use_scale_estimate_(use_scale_estimate),
      max_optimization_time_(max_optimization_time),
      output_directory_(output_directory) {
  // set imu preintegration params
  nlohmann::json J;
  beam::ReadJson(imu_intrinsics_path, J);

  imu_params_.cov_gyro_noise =
      Eigen::Matrix3d::Identity() * J["cov_gyro_noise"];
  imu_params_.cov_accel_noise =
      Eigen::Matrix3d::Identity() * J["cov_accel_noise"];
  imu_params_.cov_gyro_bias = Eigen::Matrix3d::Identity() * J["cov_gyro_bias"];
  imu_params_.cov_accel_bias =
      Eigen::Matrix3d::Identity() * J["cov_accel_bias"];

  // create optimzation graph
  local_graph_ = std::make_shared<fuse_graphs::HashGraph>();

  // create visual map
  visual_map_ = std::make_shared<vision::VisualMap>(cam_model_, local_graph_);

  // create pose refiner
  pose_refiner_ = std::make_shared<beam_cv::PoseRefinement>(1e-3);

  // make subscriber for init path
  ros::NodeHandle n;
  path_subscriber_ =
      n.subscribe(path_topic, 10, &VIOInitialization::ProcessInitPath, this);
}

bool VIOInitialization::AddImage(ros::Time cur_time) {
  frame_times_.push_back(cur_time.toNSec());
  if (init_path_) {
    ROS_INFO("Attempting VIO Initialization.");

    // Build frame vectors
    BuildFrameVectors();

    // Initialize imu preintegration
    PerformIMUInitialization(valid_frames_);
    imu_preint_ =
        std::make_shared<bs_models::ImuPreintegration>(imu_params_, bg_, ba_);

    // Align poses to world gravity
    Eigen::Quaterniond q =
        Eigen::Quaterniond::FromTwoVectors(gravity_, GRAVITY_WORLD);
    for (auto &f : valid_frames_) {
      f.q = q * f.q;
      f.p = q * f.p;
      // Apply scale estimate if desired
      if (use_scale_estimate_)
        f.p = scale_ * f.p;
    }

    // Add poses from path and imu constraints to graph
    AddPosesAndInertialConstraints(valid_frames_, true);
    OutputResults(valid_frames_);

    // Add landmarks and visual constraints to graph
    size_t init_lms = AddVisualConstraints(valid_frames_);

    // output pre optimization poses
    ROS_INFO("Frame poses before optimization:");
    OutputFramePoses(valid_frames_);

    // optimize valid frames
    OptimizeGraph();

    // localize the frames that are outside of the given path
    for (auto &f : invalid_frames_) {
      Eigen::Matrix4d T_WORLD_BASELINK;
      // if failure to localize next frame then init is a failure
      if (!LocalizeFrame(f, T_WORLD_BASELINK)) {
        return false;
      }
      beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_BASELINK, f.q,
                                                      f.p);
    }

    // add localized poses and imu constraints
    AddPosesAndInertialConstraints(invalid_frames_, false);

    // add landmarks and visual constraints for the invalid frames
    init_lms += AddVisualConstraints(invalid_frames_);

    // optimize with invalid frames
    OptimizeGraph();

    // output post optimization poses
    ROS_INFO("Frame poses after optimization:");
    OutputFramePoses(valid_frames_);
    OutputFramePoses(invalid_frames_);
    OutputResults(valid_frames_);

    // log initialization statistics
    ROS_INFO("Initialized Map Points: %zu", init_lms);
    is_initialized_ = true;
  }
  return is_initialized_;
}

void VIOInitialization::AddIMU(const sensor_msgs::Imu &msg) {
  imu_buffer_.push(msg);
}

void VIOInitialization::ProcessInitPath(
    const InitializedPathMsg::ConstPtr &msg) {
  init_path_ = std::make_shared<InitializedPathMsg>();
  *init_path_ = *msg;
}

bool VIOInitialization::Initialized() { return is_initialized_; }

const fuse_graphs::HashGraph &VIOInitialization::GetGraph() {
  return *local_graph_;
}

std::shared_ptr<bs_models::ImuPreintegration>
VIOInitialization::GetPreintegrator() {
  return imu_preint_;
}

void VIOInitialization::BuildFrameVectors() {
  ros::Time start = init_path_->poses[0].header.stamp;
  ros::Time end = init_path_->poses[init_path_->poses.size() - 1].header.stamp;
  valid_frames_.clear();
  invalid_frames_.clear();

  // get start and end time of input path
  for (auto &kf : frame_times_) {
    ros::Time stamp;
    stamp.fromNSec(kf);
    if (stamp < start)
      continue;

    // add imu data to frames preintegrator
    PreIntegrator preintegrator;
    preintegrator.cov_w = imu_params_.cov_gyro_noise;
    preintegrator.cov_a = imu_params_.cov_accel_noise;
    preintegrator.cov_bg = imu_params_.cov_gyro_bias;
    preintegrator.cov_ba = imu_params_.cov_accel_bias;
    while (imu_buffer_.front().header.stamp <= stamp && !imu_buffer_.empty()) {
      IMUData imu_data(imu_buffer_.front());
      preintegrator.data.push_back(imu_data);
      imu_buffer_.pop();
    }

    if (stamp <= end) {
      // get pose of frame using path
      Eigen::Matrix4d T_WORLD_BASELINK;
      InterpolateTransformFromPath(init_path_->poses, stamp, T_WORLD_BASELINK);
      Eigen::Vector3d p_WORLD_BASELINK;
      Eigen::Quaterniond q_WORLD_BASELINK;
      beam::TransformMatrixToQuaternionAndTranslation(
          T_WORLD_BASELINK, q_WORLD_BASELINK, p_WORLD_BASELINK);

      // create frame and add to valid frame vector
      Frame new_frame{stamp, p_WORLD_BASELINK, q_WORLD_BASELINK, preintegrator};
      valid_frames_.push_back(new_frame);
    } else {
      // get arbitrary pose values
      Eigen::Vector3d p_WORLD_BASELINK(0, 0, 0);
      Eigen::Quaterniond q_WORLD_BASELINK(0, 0, 0, 0);

      // create frame and add to invalid frame vector
      Frame new_frame{stamp, p_WORLD_BASELINK, q_WORLD_BASELINK, preintegrator};
      invalid_frames_.push_back(new_frame);
    }
  }
}

void VIOInitialization::PerformIMUInitialization(std::vector<Frame> &frames) {
  // estimate gyroscope bias
  if (init_path_->gyroscope_bias.x == 0 && init_path_->gyroscope_bias.y == 0 &&
      init_path_->gyroscope_bias.z == 0) {
    SolveGyroBias(frames);
  } else {
    bg_ << init_path_->gyroscope_bias.x, init_path_->gyroscope_bias.y,
        init_path_->gyroscope_bias.z;
  }

  // estimate gravity and scale
  if (init_path_->gravity.x == 0 && init_path_->gravity.y == 0 &&
      init_path_->gravity.z == 0) {
    SolveGravityAndScale(frames);
    RefineGravityAndScale(frames);
  } else {
    gravity_ << init_path_->gravity.x, init_path_->gravity.y,
        init_path_->gravity.z;
    scale_ = init_path_->scale;
  }

  // estimate accelerometer bias
  if (init_path_->accelerometer_bias.x == 0 &&
      init_path_->accelerometer_bias.y == 0 &&
      init_path_->accelerometer_bias.z == 0) {
    SolveAccelBias(frames);
  } else {
    ba_ << init_path_->accelerometer_bias.x, init_path_->accelerometer_bias.y,
        init_path_->accelerometer_bias.z;
  }
}

void VIOInitialization::AddPosesAndInertialConstraints(
    const std::vector<Frame> &frames, bool set_start) {
  // add initial poses and imu data to preintegrator
  for (int i = 0; i < frames.size(); i++) {
    // Add frame's pose to graph
    Frame frame = frames[i];
    visual_map_->AddPosition(frame.p, frame.t);
    visual_map_->AddOrientation(frame.q, frame.t);
    Eigen::Matrix4d T;
    beam::QuaternionAndTranslationToTransformMatrix(frame.q, frame.p, T);

    // Push its imu messages
    for (auto &imu_data : frame.preint.data) {
      imu_preint_->AddToBuffer(imu_data);
    }

    fuse_variables::Orientation3DStamped::SharedPtr img_orientation =
        visual_map_->GetOrientation(frame.t);
    fuse_variables::Position3DStamped::SharedPtr img_position =
        visual_map_->GetPosition(frame.t);

    // // Add respective imu constraints
    // if (set_start && i == 0) {
    //   // estimate velocity at frame.t
    //   Eigen::Vector3d velocity_vec;
    //   EstimateVelocityFromPath(init_path_->poses, frame.t, velocity_vec);
    //   // make fuse variable
    //   fuse_variables::VelocityLinear3DStamped::SharedPtr velocity =
    //       std::make_shared<fuse_variables::VelocityLinear3DStamped>(frame.t);
    //   velocity->x() = velocity_vec[0];
    //   velocity->y() = velocity_vec[1];
    //   velocity->z() = velocity_vec[2];
    //   ROS_INFO("Initial velocity:");
    //   std::cout << velocity_vec << std::endl;
    //   imu_preint_->SetStart(frame.t, img_orientation, img_position,
    //   velocity);
    // } else {
    //   // get imu transaction
    //   fuse_core::Transaction::SharedPtr transaction =
    //       imu_preint_->RegisterNewImuPreintegratedFactor(
    //           frame.t, img_orientation, img_position);
    //   // update graph with the transaction
    //   local_graph_->update(*transaction);
    // }
  }
}

size_t
VIOInitialization::AddVisualConstraints(const std::vector<Frame> &frames) {
  ros::Time start = frames[0].t, end = frames[frames.size() - 1].t;
  size_t num_landmarks = 0;

  // get all landmarks in the window
  std::vector<uint64_t> landmarks =
      tracker_->GetLandmarkIDsInWindow(start, end);
  for (auto &id : landmarks) {
    fuse_variables::Point3DLandmark::SharedPtr lm =
        visual_map_->GetLandmark(id);
    if (lm) {
      // if the landmark already exists then add constraint
      for (auto &f : frames) {
        try {
          visual_map_->AddConstraint(f.t, id, tracker_->Get(f.t, id));
        } catch (const std::out_of_range &oor) {
        }
      }
    } else {
      // otherwise then triangulate then add the constraints
      std::vector<Eigen::Matrix4d, beam::AlignMat4d> T_cam_world_v;
      std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
      std::vector<ros::Time> observation_stamps;
      beam_cv::FeatureTrack track = tracker_->GetTrack(id);
      for (auto &m : track) {
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

bool VIOInitialization::LocalizeFrame(const Frame &frame,
                                      Eigen::Matrix4d &T_WORLD_BASELINK) {
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  std::vector<uint64_t> landmarks = tracker_->GetLandmarkIDsInImage(frame.t);

  // get 2d-3d correspondences
  for (auto &id : landmarks) {
    fuse_variables::Point3DLandmark::SharedPtr lm =
        visual_map_->GetLandmark(id);
    if (lm) {
      Eigen::Vector2i pixeli = tracker_->Get(frame.t, id).cast<int>();
      pixels.push_back(pixeli);
      Eigen::Vector3d point(lm->x(), lm->y(), lm->z());
      points.push_back(point);
    }
  }

  if (points.size() < 15) {
    return false;
  }

  // estimate with ransac pnp
  Eigen::Matrix4d T_CAMERA_WORLD_est =
      beam_cv::AbsolutePoseEstimator::RANSACEstimator(cam_model_, pixels,
                                                      points);

  // refine pose using reprojection error
  Eigen::Matrix4d T_CAMERA_WORLD_ref =
      pose_refiner_->RefinePose(T_CAMERA_WORLD_est, cam_model_, pixels, points);
  Eigen::Matrix4d T_WORLD_CAMERA = T_CAMERA_WORLD_ref.inverse();
  extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_);

  // transform into baselink frame
  T_WORLD_BASELINK = T_WORLD_CAMERA * T_cam_baselink_;
  return true;
}

void VIOInitialization::OptimizeGraph() {
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

void VIOInitialization::SolveGyroBias(std::vector<Frame> &frames) {
  for (size_t i = 0; i < frames.size(); i++) {
    frames[i].preint.Integrate(frames[i].t, bg_, ba_, true, false);
  }
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();

  for (size_t j = 1; j < frames.size(); ++j) {
    const size_t i = j - 1;

    Frame frame_i = frames[i];
    Frame frame_j = frames[j];

    const Eigen::Quaterniond &dq = frames[j].preint.delta.q;
    const Eigen::Matrix3d &dq_dbg = frames[j].preint.jacobian.dq_dbg;
    A += dq_dbg.transpose() * dq_dbg;

    Eigen::Quaterniond tmp = (frames[i].q * dq).conjugate() * frames[j].q;
    Eigen::Matrix3d tmp_R = tmp.normalized().toRotationMatrix();

    b += dq_dbg.transpose() * beam::RToLieAlgebra(tmp_R);
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  bg_ = svd.solve(b);
}

void VIOInitialization::SolveAccelBias(std::vector<Frame> &frames) {
  if (gravity_.isZero(1e-9)) {
    ROS_WARN(
        "Can't estimate acceleration bias without first estimating gravity.");
    return;
  }
  for (size_t i = 0; i < frames.size(); i++) {
    frames[i].preint.Integrate(frames[i].t, bg_, ba_, true, false);
  }
  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
  Eigen::Vector4d b = Eigen::Vector4d::Zero();

  for (size_t j = 1; j + 1 < frames.size(); ++j) {
    const size_t i = j - 1;
    const size_t k = j + 1;

    const bs_common::Delta &delta_ij = frames[j].preint.delta;
    const bs_common::Delta &delta_jk = frames[k].preint.delta;
    const bs_common::Jacobian &jacobian_ij = frames[j].preint.jacobian;
    const bs_common::Jacobian &jacobian_jk = frames[k].preint.jacobian;

    Eigen::Matrix<double, 3, 4> C;
    C.block<3, 1>(0, 0) = delta_ij.t.toSec() * (frames[k].p - frames[j].p) -
                          delta_jk.t.toSec() * (frames[j].p - frames[i].p);
    C.block<3, 3>(0, 1) =
        -(frames[j].q * jacobian_jk.dp_dba * delta_ij.t.toSec() +
          frames[i].q * jacobian_ij.dv_dba * delta_ij.t.toSec() *
              delta_jk.t.toSec() -
          frames[i].q * jacobian_ij.dp_dba * delta_jk.t.toSec());
    Eigen::Vector3d d =
        0.5 * delta_ij.t.toSec() * delta_jk.t.toSec() *
            (delta_ij.t.toSec() + delta_jk.t.toSec()) * gravity_ +
        delta_ij.t.toSec() * (frames[j].q * delta_jk.p) +
        delta_ij.t.toSec() * delta_jk.t.toSec() * (frames[i].q * delta_ij.v) -
        delta_jk.t.toSec() * (frames[i].q * delta_ij.p);
    A += C.transpose() * C;
    b += C.transpose() * d;
  }

  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Vector4d x = svd.solve(b);
  ba_ = x.segment<3>(1);
}

void VIOInitialization::SolveGravityAndScale(std::vector<Frame> &frames) {
  for (size_t i = 0; i < frames.size(); i++) {
    frames[i].preint.Integrate(frames[i].t, bg_, ba_, true, false);
  }
  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
  Eigen::Vector4d b = Eigen::Vector4d::Zero();

  for (size_t j = 1; j + 1 < frames.size(); ++j) {
    const size_t i = j - 1;
    const size_t k = j + 1;

    const bs_common::Delta &delta_ij = frames[j].preint.delta;
    const bs_common::Delta &delta_jk = frames[k].preint.delta;
    const bs_common::Jacobian &jacobian_ij = frames[j].preint.jacobian;
    const bs_common::Jacobian &jacobian_jk = frames[k].preint.jacobian;

    Eigen::Matrix<double, 3, 4> C;
    C.block<3, 3>(0, 0) = -0.5 * delta_ij.t.toSec() * delta_jk.t.toSec() *
                          (delta_ij.t.toSec() + delta_jk.t.toSec()) *
                          Eigen::Matrix3d::Identity();
    C.block<3, 1>(0, 3) = delta_ij.t.toSec() * (frames[k].p - frames[j].p) -
                          delta_jk.t.toSec() * (frames[j].p - frames[i].p);
    Eigen::Vector3d d =
        delta_ij.t.toSec() * (frames[j].q * delta_jk.p) +
        delta_ij.t.toSec() * delta_jk.t.toSec() * (frames[i].q * delta_ij.v) -
        delta_jk.t.toSec() * (frames[i].q * delta_ij.p);
    A += C.transpose() * C;
    b += C.transpose() * d;
  }

  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Vector4d x = svd.solve(b);
  gravity_ = x.segment<3>(0).normalized() * GRAVITY_NOMINAL;
  scale_ = x(3);
}

void VIOInitialization::RefineGravityAndScale(std::vector<Frame> &frames) {
  static const double damp = 0.1;
  for (size_t i = 0; i < frames.size(); i++) {
    frames[i].preint.Integrate(frames[i].t, bg_, ba_, true, false);
  }
  int N = (int)frames.size();
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  Eigen::VectorXd x;
  A.resize((N - 1) * 6, 2 + 1 + 3 * N);
  b.resize((N - 1) * 6);
  x.resize(2 + 1 + 3 * N);

  for (size_t iter = 0; iter < 5; ++iter) {
    A.setZero();
    b.setZero();
    Eigen::Matrix<double, 3, 2> Tg = s2_tangential_basis(gravity_);

    for (size_t j = 1; j < frames.size(); ++j) {
      const size_t i = j - 1;

      const bs_common::Delta &delta = frames[j].preint.delta;

      A.block<3, 2>(i * 6, 0) = -0.5 * delta.t.toSec() * delta.t.toSec() * Tg;
      A.block<3, 1>(i * 6, 2) = frames[j].p - frames[i].p;
      A.block<3, 3>(i * 6, 3 + i * 3) =
          -delta.t.toSec() * Eigen::Matrix3d::Identity();
      b.segment<3>(i * 6) = 0.5 * delta.t.toSec() * delta.t.toSec() * gravity_ +
                            frames[i].q * delta.p;

      A.block<3, 2>(i * 6 + 3, 0) = -delta.t.toSec() * Tg;
      A.block<3, 3>(i * 6 + 3, 3 + i * 3) = -Eigen::Matrix3d::Identity();
      A.block<3, 3>(i * 6 + 3, 3 + j * 3) = Eigen::Matrix3d::Identity();
      b.segment<3>(i * 6 + 3) =
          delta.t.toSec() * gravity_ + frames[i].q * delta.v;
    }

    x = A.fullPivHouseholderQr().solve(b);
    Eigen::Vector2d dg = x.segment<2>(0);
    gravity_ = (gravity_ + damp * Tg * dg).normalized() * GRAVITY_NOMINAL;
  }
  scale_ = x(2);
}

void VIOInitialization::OutputFramePoses(const std::vector<Frame> &frames) {
  for (auto &f : frames) {
    std::cout << f.t << std::endl;
    std::cout << visual_map_->GetBaselinkPose(f.t) << std::endl;
  }
}

void VIOInitialization::OutputResults(const std::vector<Frame> &frames) {
  if (!boost::filesystem::exists(output_directory_) ||
      output_directory_.empty()) {
    ROS_WARN("Output directory does not exist or is empty, not outputting VIO "
             "Initializer results.");
  } else {
    // add frame poses to cloud and save
    pcl::PointCloud<pcl::PointXYZRGB> frame_cloud;
    for (auto &f : frames) {
      frame_cloud = beam::AddFrameToCloud(
          frame_cloud, visual_map_->GetCameraPose(f.t).value(), 0.001);
    }

    // add all landmark points to cloud and save
    std::vector<uint64_t> landmarks = tracker_->GetLandmarkIDsInWindow(
        frames[0].t, frames[frames.size() - 1].t);
    pcl::PointCloud<pcl::PointXYZ> points_cloud;
    for (auto &id : landmarks) {
      fuse_variables::Point3DLandmark::SharedPtr lm =
          visual_map_->GetLandmark(id);
      if (lm) {
        pcl::PointXYZ p(lm->x(), lm->y(), lm->z());
        points_cloud.push_back(p);
      }
    }

    std::string error_message{};
    if (!beam::SavePointCloud<pcl::PointXYZRGB>(
            output_directory_ + "/frames.pcd", frame_cloud,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
    if (!beam::SavePointCloud<pcl::PointXYZ>(
            output_directory_ + "/points.pcd", points_cloud,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
  }
}
}
} // namespace bs_models::vision
