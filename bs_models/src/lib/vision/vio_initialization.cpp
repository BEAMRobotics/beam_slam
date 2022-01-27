#include <bs_models/vision/vio_initialization.h>

#include <nlohmann/json.hpp>

#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/math.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

#include <bs_common/utils.h>

namespace bs_models { namespace vision {

VIOInitialization::VIOInitialization(
    std::shared_ptr<beam_calibration::CameraModel> cam_model,
    std::shared_ptr<beam_cv::Tracker> tracker, const std::string& path_topic,
    const std::string& imu_intrinsics_path, bool use_scale_estimate,
    double max_optimization_time, const std::string& output_directory)
    : cam_model_(cam_model),
      tracker_(tracker),
      use_scale_estimate_(use_scale_estimate),
      max_optimization_time_(max_optimization_time),
      output_directory_(output_directory) {
  // set imu preintegration params
  nlohmann::json J;
  beam::ReadJson(imu_intrinsics_path, J);
  imu_params_.cov_prior_noise = J["cov_prior_noise"];
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
  visual_map_ = std::make_shared<vision::VisualMap>(cam_model_);
  // initialize pose refiner object with params
  pose_refiner_ = std::make_shared<beam_cv::PoseRefinement>(1e-2);
  // make subscriber for init path
  ros::NodeHandle n;
  path_subscriber_ =
      n.subscribe(path_topic, 10, &VIOInitialization::ProcessInitPath, this);
}

bool VIOInitialization::AddImage(const ros::Time& cur_time) {
  frame_times_.push_back(cur_time.toNSec());
  if (init_path_) {
    ROS_INFO("Attempting VIO Initialization.");

    // prune poses in path that come before any imu messages
    while (init_path_->poses[0].header.stamp <
           imu_buffer_.front().header.stamp) {
      init_path_->poses.erase(init_path_->poses.begin());
    }

    // Estimate imu biases and gravity using the initial path
    bs_models::ImuPreintegration::EstimateParameters(*init_path_, imu_buffer_,
                                                     imu_params_, gravity_, bg_,
                                                     ba_, velocities_, scale_);

    // if the scale estimate isnt valid then retry
    if (use_scale_estimate_ && (scale_ < 0.02 || scale_ > 1.0)) {
      // TODO: publish reset request to reinitialize
      ROS_FATAL_STREAM("Invalid scale estimate: " << scale_
                                                  << ", reinitializing.");
      std::queue<sensor_msgs::Imu> empty;
      std::swap(imu_buffer_, empty);
      frame_times_.clear();
      return false;
    }
    
    // initialize preintegration
    imu_preint_ =
        std::make_shared<bs_models::ImuPreintegration>(imu_params_, bg_, ba_);
    ROS_DEBUG_STREAM("Estimated IMU Parameters: \n"
                     << "Accel Bias:\n"
                     << ba_ << "\nGyro Bias:\n"
                     << bg_ << "\nGravity:\n"
                     << gravity_ << "\nScale:" << scale_);

    // Build frame vectors
    BuildFrameVectors();

    // Align poses to world gravity
    AlignPosesToGravity();

    // Add poses from path and imu constraints to graph
    AddPosesAndInertialConstraints(valid_frames_, true);

    // Add landmarks and visual constraints to graph
    size_t init_lms = AddVisualConstraints(valid_frames_);

    if (invalid_frames_.size() > 0) {
      for (auto& f : invalid_frames_) {
        // Localize frame and add to graph
        AddPosesAndInertialConstraints({f}, false);
        // add landmarks and visual constraints for the invalid frames
        init_lms += AddVisualConstraints({f});
      }
    }

    // optimize graph
    OptimizeGraph();

    // output results
    OutputResults();

    // log initialization statistics
    ROS_INFO("Initialized Map Points: %zu", init_lms);

    // memory clean up
    visual_map_->Clear();
    std::queue<sensor_msgs::Imu> empty;
    std::swap(imu_buffer_, empty);
    frame_times_.clear();
    valid_frames_.clear();
    invalid_frames_.clear();

    is_initialized_ = true;
  }
  return is_initialized_;
}

void VIOInitialization::AddIMU(const sensor_msgs::Imu& msg) {
  imu_buffer_.push(msg);
}

void VIOInitialization::ProcessInitPath(
    const InitializedPathMsg::ConstPtr& msg) {
  init_path_ = std::make_shared<InitializedPathMsg>();
  *init_path_ = *msg;
}

bool VIOInitialization::Initialized() {
  return is_initialized_;
}

const fuse_core::Graph::SharedPtr& VIOInitialization::GetGraph() {
  return local_graph_;
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
  for (auto& kf : frame_times_) {
    ros::Time stamp;
    stamp.fromNSec(kf);
    if (stamp < start) continue;
    if (stamp <= end) {
      // get pose of frame using path
      Eigen::Matrix4d T_WORLD_BASELINK;
      InterpolateTransformFromPath(init_path_->poses, stamp, T_WORLD_BASELINK);
      Eigen::Vector3d p_WORLD_BASELINK;
      Eigen::Quaterniond q_WORLD_BASELINK;
      beam::TransformMatrixToQuaternionAndTranslation(
          T_WORLD_BASELINK, q_WORLD_BASELINK, p_WORLD_BASELINK);
      // create frame and add to valid frame vector
      Frame new_frame{stamp, p_WORLD_BASELINK, q_WORLD_BASELINK};
      valid_frames_.push_back(new_frame);
    } else {
      // get arbitrary pose values
      Eigen::Vector3d p_WORLD_BASELINK(0, 0, 0);
      Eigen::Quaterniond q_WORLD_BASELINK(0, 0, 0, 0);
      // create frame and add to invalid frame vector
      Frame new_frame{stamp, p_WORLD_BASELINK, q_WORLD_BASELINK};
      invalid_frames_.push_back(new_frame);
    }
  }
}

void VIOInitialization::AddPosesAndInertialConstraints(
    const std::vector<Frame>& frames, bool is_valid) {
  // add inertial constraints between poses
  for (int i = 0; i < frames.size(); i++) {
    Frame frame = frames[i];
    // add imu data to preint for this frame
    while (imu_buffer_.front().header.stamp <= frame.t &&
           !imu_buffer_.empty()) {
      imu_preint_->AddToBuffer(imu_buffer_.front());
      imu_buffer_.pop();
    }

    // if the current frame doesnt have a pose then estimate it using the imu
    if (!is_valid) {
      Eigen::Matrix4d T_WORLD_BASELINK = LocalizeFrame(frame.t);
      beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_BASELINK, frame.q,
                                                      frame.p);
    }

    // add pose to graph
    auto pose_transaction = fuse_core::Transaction::make_shared();
    pose_transaction->stamp(frame.t);
    visual_map_->AddPosition(frame.p, frame.t, pose_transaction);
    visual_map_->AddOrientation(frame.q, frame.t, pose_transaction);
    // send transaction to graph
    local_graph_->update(*pose_transaction);

    // get the fuse pose variables
    fuse_variables::Orientation3DStamped::SharedPtr img_orientation =
        visual_map_->GetOrientation(frame.t);
    fuse_variables::Position3DStamped::SharedPtr img_position =
        visual_map_->GetPosition(frame.t);

    // estimate velocity at frame if its a valid frame
    fuse_variables::VelocityLinear3DStamped::SharedPtr velocity =
        std::make_shared<fuse_variables::VelocityLinear3DStamped>(frame.t);
    if (is_valid) {
      size_t index = 0;
      for (size_t i = 0; i < init_path_->poses.size(); i++) {
        auto pose = init_path_->poses[i];
        if (pose.header.stamp > frame.t) {
          index = i - 1;
          break;
        }
      }
      Eigen::Vector3d velocity_vec = beam::InterpolateVector(
          velocities_[index], init_path_->poses[index].header.stamp.toSec(),
          velocities_[index + 1],
          init_path_->poses[index + 1].header.stamp.toSec(), frame.t.toSec());
      velocity->x() = velocity_vec[0];
      velocity->y() = velocity_vec[1];
      velocity->z() = velocity_vec[2];
    }

    // Add appropriate imu constraints
    if (is_valid && i == 0) {
      imu_preint_->SetStart(frame.t, img_orientation, img_position, velocity);
    } else if (is_valid) {
      // get imu transaction using velocity
      fuse_core::Transaction::SharedPtr imu_transaction =
          imu_preint_->RegisterNewImuPreintegratedFactor(
              frame.t, img_orientation, img_position, velocity);
      // send transaction to graph
      local_graph_->update(*imu_transaction);
    } else if (!is_valid) {
      // get imu transaction without velocity
      fuse_core::Transaction::SharedPtr imu_transaction =
          imu_preint_->RegisterNewImuPreintegratedFactor(
              frame.t, img_orientation, img_position);
      // send transaction to graph
      local_graph_->update(*imu_transaction);
    }
  }

  // update visual map with updated graph
  visual_map_->UpdateGraph(local_graph_);
}

size_t
    VIOInitialization::AddVisualConstraints(const std::vector<Frame>& frames) {
  ros::Time start = frames[0].t, end = frames[frames.size() - 1].t;
  size_t num_landmarks = 0;
  // make transaction
  auto landmark_transaction = fuse_core::Transaction::make_shared();
  landmark_transaction->stamp(end);
  // get all landmarks in the window
  std::vector<uint64_t> landmarks =
      tracker_->GetLandmarkIDsInWindow(start, end);
  for (auto& id : landmarks) {
    fuse_variables::Point3DLandmark::SharedPtr lm =
        visual_map_->GetLandmark(id);
    if (lm) {
      // if the landmark already exists then add constraint
      for (auto& f : frames) {
        try {
          visual_map_->AddConstraint(f.t, id, tracker_->Get(f.t, id),
                                     landmark_transaction);
        } catch (const std::out_of_range& oor) {}
      }
    } else {
      // otherwise then triangulate then add the constraints
      std::vector<Eigen::Matrix4d, beam::AlignMat4d> T_cam_world_v;
      std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
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
          visual_map_->AddLandmark(point.value(), id, landmark_transaction);
          for (int i = 0; i < observation_stamps.size(); i++) {
            visual_map_->AddConstraint(observation_stamps[i], id,
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

Eigen::Matrix4d VIOInitialization::LocalizeFrame(const ros::Time& img_time) {
  // get most recent extrinsics, if failure then process frame later
  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get camera to baselink transform.");
  }
  // get 2d-3d correspondences
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  std::vector<uint64_t> landmarks = tracker_->GetLandmarkIDsInImage(img_time);
  for (auto& id : landmarks) {
    fuse_variables::Point3DLandmark::SharedPtr lm =
        visual_map_->GetLandmark(id);
    if (lm) {
      Eigen::Vector3d point(lm->x(), lm->y(), lm->z());
      Eigen::Vector2i pixeli = tracker_->Get(img_time, id).cast<int>();
      pixels.push_back(pixeli);
      points.push_back(point);
    }
  }
  // get pose estimate
  Eigen::Matrix4d T_WORLD_BASELINK_inertial;
  std::shared_ptr<Eigen::Matrix<double, 6, 6>> covariance =
      std::make_shared<Eigen::Matrix<double, 6, 6>>();
  imu_preint_->GetPose(T_WORLD_BASELINK_inertial, img_time, covariance);
  // refine with visual info
  Eigen::Matrix4d T_CAMERA_WORLD_est =
      (T_WORLD_BASELINK_inertial * T_cam_baselink_.inverse()).inverse();
  Eigen::Matrix4d T_WORLD_CAMERA =
      pose_refiner_
          ->RefinePose(T_CAMERA_WORLD_est, cam_model_, pixels, points,
                       covariance)
          .inverse();
  Eigen::Matrix4d T_WORLD_BASELINK = T_WORLD_CAMERA * T_cam_baselink_;
  return T_WORLD_BASELINK;
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

void VIOInitialization::AlignPosesToGravity() {
  if (gravity_.isZero(1e-9)) {
    ROS_WARN("Can't align poses to gravity as it has not been estimated yet.");
    return;
  }
  // estimate rotation from estimated gravity to world gravity
  Eigen::Quaterniond q =
      Eigen::Quaterniond::FromTwoVectors(gravity_, GRAVITY_WORLD);

  // apply rotation to initial path
  for (auto& pose : init_path_->poses) {
    Eigen::Matrix4d T;
    bs_common::PoseMsgToTransformationMatrix(pose, T);
    Eigen::Quaterniond ori;
    Eigen::Vector3d pos;
    beam::TransformMatrixToQuaternionAndTranslation(T, ori, pos);
    ori = q * ori;
    pos = q * pos;
    beam::QuaternionAndTranslationToTransformMatrix(ori, pos, T);
    bs_common::TransformationMatrixToPoseMsg(T, pose.header.stamp, pose);
  }

  // apply rotation to velocities
  for (auto& vel : velocities_) { vel = q * vel; }

  // apply rotation to valid frames
  for (auto& f : valid_frames_) {
    f.q = q * f.q;
    f.p = q * f.p;
    // Apply scale estimate if desired
    if (use_scale_estimate_) f.p = scale_ * f.p;
  }
}

void VIOInitialization::OutputResults() {
  // print results to stdout
  for (auto& f : valid_frames_) {
    beam::opt<Eigen::Matrix4d> T = visual_map_->GetBaselinkPose(f.t);
    if (T.has_value()) {
      ROS_INFO("Initialization Keyframe Time: %f", f.t.toSec());
      ROS_INFO("%s", beam::TransformationMatrixToString(T.value()).c_str());
    }
  }
  for (auto& f : invalid_frames_) {
    beam::opt<Eigen::Matrix4d> T = visual_map_->GetBaselinkPose(f.t);
    if (T.has_value()) {
      ROS_INFO("Initialization Keyframe Time: %f", f.t.toSec());
      ROS_INFO("%s", beam::TransformationMatrixToString(T.value()).c_str());
    }
  }
  // output results to point cloud in specified path
  if (!boost::filesystem::exists(output_directory_) ||
      output_directory_.empty()) {
    ROS_WARN("Output directory does not exist or is empty, not outputting VIO "
             "Initializer results.");
  } else {
    // add frame poses to cloud and save
    pcl::PointCloud<pcl::PointXYZRGB> frame_cloud;
    for (auto& f : valid_frames_) {
      beam::opt<Eigen::Matrix4d> T = visual_map_->GetBaselinkPose(f.t);
      if (T.has_value()) {
        frame_cloud = beam::AddFrameToCloud(frame_cloud, T.value(), 0.001);
      }
    }
    for (auto& f : invalid_frames_) {
      beam::opt<Eigen::Matrix4d> T = visual_map_->GetBaselinkPose(f.t);
      if (T.has_value()) {
        frame_cloud = beam::AddFrameToCloud(frame_cloud, T.value(), 0.001);
      }
    }

    // add all landmark points to cloud and save
    std::vector<uint64_t> landmarks = tracker_->GetLandmarkIDsInWindow(
        valid_frames_[0].t, invalid_frames_[invalid_frames_.size() - 1].t);
    pcl::PointCloud<pcl::PointXYZ> points_cloud;
    for (auto& id : landmarks) {
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

}} // namespace bs_models::vision
