#include <bs_models/visual_odometry.h>

#include <pluginlib/class_list_macros.h>

#include <fuse_core/transaction.h>
#include <fuse_variables/acceleration_linear_3d_stamped.h>
#include <fuse_variables/velocity_angular_3d_stamped.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Time.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/Utils.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_utils/pointclouds.h>

#include <bs_common/conversions.h>
#include <bs_common/graph_access.h>
#include <bs_constraints/inertial/absolute_imu_state_3d_stamped_constraint.h>
#include <bs_constraints/visual/euclidean_reprojection_constraint.h>
#include <bs_models/graph_visualization/helpers.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::VisualOdometry, fuse_core::SensorModel)

namespace bs_models {

using namespace vision;

VisualOdometry::VisualOdometry()
    : fuse_core::AsyncSensorModel(3),
      device_id_(fuse_core::uuid::NIL),
      throttled_measurement_callback_(std::bind(
          &VisualOdometry::processMeasurements, this, std::placeholders::_1)) {}

void VisualOdometry::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  vo_params_.loadFromROS(private_node_handle_);
  calibration_params_.loadFromROS();

  // setup publishers
  reset_publisher_ =
      private_node_handle_.advertise<std_msgs::Empty>("/local_mapper/reset", 1);
  odometry_publisher_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odometry", 100);
  keyframe_publisher_ =
      private_node_handle_.advertise<geometry_msgs::PoseStamped>("pose", 10);
  imu_constraint_trigger_publisher_ =
      private_node_handle_.advertise<std_msgs::Time>(
          "/local_mapper/inertial_odometry/trigger", 10);
  slam_chunk_publisher_ =
      private_node_handle_.advertise<bs_common::SlamChunkMsg>(
          "/local_mapper/slam_results", 100);
  camera_landmarks_publisher_ =
      private_node_handle_.advertise<sensor_msgs::PointCloud2>(
          "camera_landmarks", 10);

  // create frame initializer
  frame_initializer_ = std::make_unique<bs_models::FrameInitializer>(
      vo_params_.frame_initializer_config);

  // Load camera model and create visua map object
  cam_model_ = beam_calibration::CameraModel::Create(
      calibration_params_.cam_intrinsics_path);
  cam_intrinsic_matrix_ = cam_model_->GetRectifiedModel()->GetIntrinsicMatrix();
  cv::Mat K(3, 3, CV_32F);
  cv::eigen2cv(cam_intrinsic_matrix_, K);
  K_ = K;

  // create visual map
  bool use_online_calib_for_reproj_constraints =
      vo_params_.use_online_calibration && !vo_params_.use_standalone_vo;
  visual_map_ = std::make_shared<VisualMap>(
      name(), cam_model_, vo_params_.reprojection_loss,
      vo_params_.reprojection_information_weight,
      use_online_calib_for_reproj_constraints, false);

  // local map matching stuff
  image_db_ = std::make_shared<beam_cv::ImageDatabase>();
  landmark_projection_mask_ = cv::Mat(
      cam_model_->GetWidth(), cam_model_->GetHeight(), CV_8UC1, cv::Scalar(0));

  // Initialize landmark measurement container
  landmark_container_ = std::make_shared<beam_containers::LandmarkContainer>();

  // create pose refiner for motion only BA
  pose_refiner_ = std::make_shared<beam_cv::PoseRefinement>(0.02, true, 0.2);
  validator_ = std::make_shared<vision::VOLocalizationValidation>();

  // get extrinsics
  extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_);
  T_baselink_cam_ = beam::InvertTransform(T_cam_baselink_);

  // compute the max container size
  bs_parameters::getParamRequired(ros::NodeHandle("~"), "lag_duration",
                                  lag_duration_);
  max_container_size_ = calibration_params_.camera_hz * (lag_duration_ + 1);

  // create local solver options
  if (vo_params_.use_standalone_vo) {
    // setup solve params
    ceres::Solver::Options options;
    options.minimizer_type = ceres::TRUST_REGION;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = std::thread::hardware_concurrency() / 2;
    options.max_num_iterations = 10;
    local_solver_options_ = options;
  }
}

void VisualOdometry::onStart() {
  ROS_INFO_STREAM("Starting: " << name());
  // setup subscriber
  measurement_subscriber_ =
      private_node_handle_.subscribe<bs_common::CameraMeasurementMsg>(
          ros::names::resolve("/feature_tracker/visual_measurements"), 10,
          &ThrottledMeasurementCallback::callback,
          &throttled_measurement_callback_,
          ros::TransportHints().tcpNoDelay(false));
}

void VisualOdometry::onStop() {
  ROS_INFO_STREAM("Stopping: " << name());
  shutdown();
}

void VisualOdometry::processMeasurements(
    const bs_common::CameraMeasurementMsg::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE(
      "VisualOdometry received VISUAL measurements: " << msg->header.stamp);

  // add measurements to local container
  AddMeasurementsToContainer(msg);

  // buffer the message
  std::unique_lock<std::mutex> lk(buffer_mutex_);
  visual_measurement_buffer_.push_back(msg);

  // don't process until we have initialized
  if (!is_initialized_) { return; }

  while (!visual_measurement_buffer_.empty()) {
    beam::HighResolutionTimer timer;
    // retrieve and process the message at the front of the buffer
    const auto current_msg = visual_measurement_buffer_.front();
    const auto success = ComputeOdometryAndExtendMap(current_msg);

    // buffer frame if localization fails (only if frame init isnt caught up)
    if (!success) { break; }

    visual_measurement_buffer_.pop_front();
    ROS_DEBUG_STREAM("Frame processing time: " << timer.elapsed());
  }

  // remove measurements from container if we are over the limit
  while (landmark_container_->NumImages() > max_container_size_) {
    landmark_container_->PopFront();
  }
}

bool VisualOdometry::ComputeOdometryAndExtendMap(
    const bs_common::CameraMeasurementMsg::ConstPtr& msg) {
  const ros::Time timestamp = msg->header.stamp;
  // estimate pose of frame wrt current graph
  Eigen::Matrix4d T_WORLD_BASELINK;
  Eigen::Matrix<double, 6, 6> covariance;
  if (!LocalizeFrame(timestamp, T_WORLD_BASELINK, covariance)) { return false; }

  // publish odometry
  PublishOdometry(timestamp, T_WORLD_BASELINK, covariance);

  // if not keyframe -> add to current keyframe sub trajectory
  if (!IsKeyframe(timestamp, T_WORLD_BASELINK)) {
    Eigen::Matrix4d T_WORLD_BASELINKprevkf =
        visual_map_->GetBaselinkPose(previous_keyframe_).value();
    Eigen::Matrix4d T_KEYFRAME_FRAME =
        beam::InvertTransform(T_WORLD_BASELINKprevkf) * T_WORLD_BASELINK;
    keyframes_.at(previous_keyframe_).AddPose(timestamp, T_KEYFRAME_FRAME);
    return true;
  }

  // create new keyframe
  ROS_DEBUG_STREAM("VisualOdometry: New keyframe detected at: " << timestamp);
  Keyframe kf(*msg);
  keyframes_.insert({timestamp, kf});

  // extend existing map and add constraints
  ExtendMap(timestamp, T_WORLD_BASELINK, covariance);

  // publish keyframe pose
  PublishPose(timestamp, T_WORLD_BASELINK);

  // update previous keyframe time only after extending map
  previous_keyframe_ = timestamp;

  // send IO trigger
  if (vo_params_.trigger_inertial_odom_constraints) {
    std_msgs::Time time_msg;
    time_msg.data = timestamp;
    imu_constraint_trigger_publisher_.publish(time_msg);
    imu_constraint_trigger_counter_++;
  }

  return true;
}

bool VisualOdometry::LocalizeFrame(const ros::Time& timestamp,
                                   Eigen::Matrix4d& T_WORLD_BASELINK,
                                   Eigen::Matrix<double, 6, 6>& covariance) {
  std::string error;
  Eigen::Matrix4d T_WORLD_BASELINKcur;
  if (!GetInitialPoseEstimate(timestamp, T_WORLD_BASELINKcur)) { return false; }

  // get 2d-3d correspondences
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  GetPixelPointPairs(timestamp, pixels, points);

  // perform visual refinement
  if (pixels.size() >= vo_params_.required_points_to_refine) {
    if (track_lost_) { track_lost_ = false; }
    // get initial estimate in camera frame
    Eigen::Matrix4d T_CAMERA_WORLD_est = beam::InvertTransform(
        T_WORLD_BASELINKcur * beam::InvertTransform(T_cam_baselink_));

    // perform non-linear pose refinement
    Eigen::Matrix4d T_CAMERA_WORLD_ref = T_CAMERA_WORLD_est;
    bool passed_refinement{true};
    try {
      auto out_covariance = std::make_shared<Eigen::Matrix<double, 6, 6>>();
      T_CAMERA_WORLD_ref =
          pose_refiner_->RefinePose(T_CAMERA_WORLD_est, cam_model_, pixels,
                                    points, nullptr, out_covariance);
      // reorder covariance to be x, y, z, roll, pitch, yaw
      covariance.block<3, 3>(0, 0) = out_covariance->block<3, 3>(3, 3);
      covariance.block<3, 3>(0, 3) = out_covariance->block<3, 3>(3, 0);
      covariance.block<3, 3>(3, 0) = out_covariance->block<3, 3>(0, 3);
      covariance.block<3, 3>(3, 3) = out_covariance->block<3, 3>(0, 0);
    } catch (const std::runtime_error& re) { passed_refinement = false; }

    // compute baselink pose
    T_WORLD_BASELINK =
        beam::InvertTransform(T_CAMERA_WORLD_ref) * T_cam_baselink_;

    // validate localization
    bool passed_localization{true};
    if (passed_refinement) {
      const double avg_reprojection =
          ComputeAverageReprojection(T_WORLD_BASELINK, pixels, points);
      const Eigen::Matrix4d T_init_refined =
          beam::InvertTransform(T_WORLD_BASELINKcur) * T_WORLD_BASELINK;
      passed_localization =
          validator_->Validate(T_init_refined, covariance, avg_reprojection);
    }

    // fallback to frame init if failed localization
    if (!passed_localization || !passed_refinement) {
      track_lost_ = true;
      T_WORLD_BASELINK = T_WORLD_BASELINKcur;
      covariance = vo_params_.invalid_localization_covariance_weight *
                   Eigen::Matrix<double, 6, 6>::Identity();
      num_loc_fails_in_a_row_++;
    } else {
      num_loc_fails_in_a_row_ = 0;
    }

  } else {
    ROS_WARN_STREAM(
        "Not enough points for visual refinement: " << pixels.size());
    T_WORLD_BASELINK = T_WORLD_BASELINKcur;
    track_lost_ = true;
    covariance = vo_params_.invalid_localization_covariance_weight *
                 Eigen::Matrix<double, 6, 6>::Identity();
    num_loc_fails_in_a_row_++;
  }

  if (num_loc_fails_in_a_row_ > 10) {
    ROS_ERROR_STREAM("Too many localization failures in a row ("
                     << num_loc_fails_in_a_row_ << "). Resetting system.");
    std_msgs::Empty reset;
    reset_publisher_.publish(reset);
    shutdown();
  }

  // update previous frame pose
  T_WORLD_BASELINKprevframe_ = T_WORLD_BASELINK;

  return true;
}

void VisualOdometry::ExtendMap(const ros::Time& timestamp,
                               const Eigen::Matrix4d& T_WORLD_BASELINK,
                               const Eigen::Matrix<double, 6, 6>& covariance) {
  // create transaction for this keyframe
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(timestamp);
  visual_map_->AddBaselinkPose(T_WORLD_BASELINK, timestamp, transaction);

  // add prior if using a frame initializer
  if (frame_initializer_ && vo_params_.prior_information_weight != 0) {
    visual_map_->AddPosePrior(timestamp, vo_params_.prior_covariance,
                              transaction);
  }

  // project all current landmarks into current image and store as
  if (vo_params_.local_map_matching) { ProjectMapPoints(T_WORLD_BASELINK); }

  // process each landmark
  const auto landmarks = landmark_container_->GetLandmarkIDsInImage(timestamp);
  for (const auto id : landmarks) {
    if (vo_params_.use_idp) {
      ProcessLandmarkIDP(id, timestamp, transaction);
    } else {
      ProcessLandmarkEUC(id, timestamp, transaction);
    }
  }

  if (vo_params_.use_standalone_vo) {
    local_graph_->update(*transaction);

    // optimize graph
    local_graph_->optimizeFor(ros::Duration(0.05), local_solver_options_);

    // send just a relative pose constraint to the main graph
    auto pose_transaction =
        CreateVisualOdometryFactor(timestamp, T_WORLD_BASELINK, covariance);
    sendTransaction(pose_transaction);

    // publish landmarks as a point cloud
    PublishLandmarkPointCloud(*local_graph_);
  } else {
    sendTransaction(transaction);
  }
}

void VisualOdometry::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  ROS_INFO_STREAM_ONCE("VisualOdometry received initial graph.");
  std::unique_lock<std::mutex> lk(buffer_mutex_);

  // update T_cam_baselink_ from the graph if it exists
  if (vo_params_.use_online_calibration) {
    auto maybe_extrinsic =
        bs_common::GetExtrinsic(*graph, extrinsics_.GetBaselinkFrameId(),
                                extrinsics_.GetCameraFrameId());
    if (maybe_extrinsic) {
      T_baselink_cam_ = maybe_extrinsic.value();
      T_cam_baselink_ = beam::InvertTransform(T_baselink_cam_);
    }
  }

  // do initial setup
  if (!is_initialized_) {
    Initialize(graph);
    is_initialized_ = true;
    return;
  }

  std::set<uint64_t> old_ids, new_ids;
  if (!vo_params_.use_standalone_vo) {
    // update visual map with main graph
    old_ids = visual_map_->GetLandmarkIDs();
    PruneKeyframes(*graph);
    visual_map_->UpdateGraph(*graph);
    new_ids = visual_map_->GetLandmarkIDs();
  } else {
    // update local graph using main graph
    old_ids = visual_map_->GetLandmarkIDs();
    MarginalizeLocalGraph(*graph);
    UpdateLocalGraph(*graph);
    PruneKeyframes(*local_graph_);
    visual_map_->UpdateGraph(*local_graph_);
    new_ids = visual_map_->GetLandmarkIDs();
  }

  // clean up new to old landmark map if its used
  if (vo_params_.local_map_matching) {
    CleanNewToOldLandmarkMap(old_ids, new_ids);
  }
}

/****************************************************/
/*                                                  */
/*                                                  */
/*                      Helpers                     */
/*                                                  */
/*                                                  */
/****************************************************/

bool VisualOdometry::IsKeyframe(const ros::Time& timestamp,
                                const Eigen::Matrix4d& T_WORLD_BASELINK) {
  if (keyframes_.empty()) { return true; }

  Eigen::Matrix4d T_PREVKF_CURFRAME;
  if (!frame_initializer_->GetRelativePose(T_PREVKF_CURFRAME,
                                           previous_keyframe_, timestamp)) {
    ROS_WARN_STREAM(
        "Unable to retrieve relative pose from last keyframe: " << timestamp);
    return false;
  }

  // compute rotation adjusted parallax
  Eigen::Matrix3d R_PREVKF_CURFRAME = T_PREVKF_CURFRAME.block<3, 3>(0, 0);
  std::vector<uint64_t> frame1_ids =
      landmark_container_->GetLandmarkIDsInImage(previous_keyframe_);
  double total_parallax = 0.0;
  int num_correspondences = 0;
  std::vector<double> parallaxes;
  for (auto& id : frame1_ids) {
    try {
      Eigen::Vector2d p1 =
          landmark_container_->GetValue(previous_keyframe_, id);
      Eigen::Vector2d p2 = landmark_container_->GetValue(timestamp, id);
      Eigen::Vector3d bp2;
      if (!cam_model_->BackProject(p2.cast<int>(), bp2)) { continue; }
      // rotate pixel from current frame to keyframe
      Eigen::Vector3d bp2_in_kf = R_PREVKF_CURFRAME * bp2;
      Eigen::Vector2d bp2_reproj;
      if (!cam_model_->ProjectPoint(bp2_in_kf, bp2_reproj)) { continue; }
      // add to total parallax
      double d = beam::distance(p1, bp2_reproj);
      total_parallax += d;
      num_correspondences++;
    } catch (const std::out_of_range& oor) {}
  }

  const double avg_parallax =
      total_parallax / static_cast<double>(num_correspondences);
  const double percent_tracked = static_cast<double>(num_correspondences) /
                                 static_cast<double>(frame1_ids.size());

  if (avg_parallax > vo_params_.keyframe_parallax) {
    return true;
  } else if (percent_tracked <= 0.5) {
    return true;
  } else if ((timestamp - previous_keyframe_).toSec() >
             ((lag_duration_ / 2.0) - 0.5)) {
    return true;
  }
  return false;
}

void VisualOdometry::AddMeasurementsToContainer(
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
                         vo_params_.track_outlier_pixel_threshold, mask);

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

beam::opt<Eigen::Vector3d>
    VisualOdometry::TriangulateLandmark(const uint64_t id,
                                        Eigen::Vector3d& average_viewing_angle,
                                        uint64_t& visual_word_id) {
  std::vector<Eigen::Matrix4d, beam::AlignMat4d> T_cam_world_v;
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> viewing_angles;
  std::vector<uint64_t> word_ids;

  beam_containers::Track track = landmark_container_->GetTrack(id);
  for (auto& m : track) {
    const auto T_camera_world = visual_map_->GetCameraPose(m.time_point);
    // check if the pose is in the graph
    if (T_camera_world.has_value()) {
      Eigen::Vector2i pixel_i = m.value.cast<int>();
      Eigen::Matrix4d T_WORLD_CAMERA =
          beam::InvertTransform(T_camera_world.value());

      pixels.push_back(pixel_i);
      T_cam_world_v.push_back(T_WORLD_CAMERA);

      if (vo_params_.local_map_matching) {
        Eigen::Vector3d bearing_cam;
        if (cam_model_->BackProject(pixel_i, bearing_cam)) {
          // compute viewing angle in world frame
          Eigen::Vector3d bearing_world =
              (T_WORLD_CAMERA * bearing_cam.homogeneous()).hnormalized();
          viewing_angles.push_back(bearing_world);
          // compute word id
          word_ids.push_back(image_db_->GetWordID(m.descriptor));
        }
      }
    }
  }

  // must have at least 2 keyframes that have seen the landmark
  if (T_cam_world_v.size() >= 2) {
    if (vo_params_.local_map_matching) {
      // compute average viewing angle
      average_viewing_angle = Eigen::Vector3d::Zero();
      for (const auto& viewing_angle : viewing_angles) {
        average_viewing_angle += viewing_angle;
      }
      average_viewing_angle /= static_cast<double>(viewing_angles.size());

      // compute mode word id
      std::map<uint64_t, int> word_id_counts;
      for (const auto& word_id : word_ids) {
        if (word_id_counts.find(word_id) == word_id_counts.end()) {
          word_id_counts[word_id] = 1;
        } else {
          word_id_counts[word_id]++;
        }
      }
      int max = 0;
      for (const auto& [id, count] : word_id_counts) {
        if (count > max) {
          visual_word_id = id;
          max = count;
        }
      }
    } else {
      average_viewing_angle = Eigen::Vector3d::Zero();
      visual_word_id = 0;
    }

    // if we've lost track, ease the requirements on new landmarks
    if (track_lost_) {
      return beam_cv::Triangulation::TriangulatePoint(cam_model_, T_cam_world_v,
                                                      pixels);
    } else {
      return beam_cv::Triangulation::TriangulatePoint(
          cam_model_, T_cam_world_v, pixels,
          vo_params_.max_triangulation_distance,
          vo_params_.max_triangulation_reprojection);
    }
  }
  return {};
}

void VisualOdometry::GetPixelPointPairs(
    const ros::Time& timestamp,
    std::vector<Eigen::Vector2i, beam::AlignVec2i>& pixels,
    std::vector<Eigen::Vector3d, beam::AlignVec3d>& points) {
  std::vector<std::pair<Eigen::Vector2i, Eigen::Vector3d>> pixel_point_pairs;
  std::vector<uint64_t> landmarks =
      landmark_container_->GetLandmarkIDsInImage(timestamp);
  for (const uint64_t id : landmarks) {
    if (vo_params_.use_idp) {
      auto lm = visual_map_->GetInverseDepthLandmark(id);
      if (lm) {
        Eigen::Vector3d camera_t_point = lm->camera_t_point();
        auto T_WORLD_CAMERA = visual_map_->GetCameraPose(lm->anchorStamp());
        if (!T_WORLD_CAMERA.has_value()) { continue; }
        Eigen::Vector3d world_t_point =
            (T_WORLD_CAMERA.value() * camera_t_point.homogeneous())
                .hnormalized();
        Eigen::Vector2i pixel =
            landmark_container_->GetValue(timestamp, id).cast<int>();
        points.push_back(world_t_point);
        pixels.push_back(pixel);
      }
    } else {
      uint64_t graph_lm_id = id;
      if (new_to_old_lm_ids_.left.find(id) != new_to_old_lm_ids_.left.end()) {
        graph_lm_id = new_to_old_lm_ids_.left.at(id);
      }

      bs_variables::Point3DLandmark::SharedPtr lm =
          visual_map_->GetLandmark(graph_lm_id);
      if (lm) {
        Eigen::Vector3d point = lm->point();
        Eigen::Vector2i pixel =
            landmark_container_->GetValue(timestamp, id).cast<int>();
        points.push_back(point);
        pixels.push_back(pixel);
      }
    }
  }
}

void VisualOdometry::Initialize(fuse_core::Graph::ConstSharedPtr graph) {
  const auto timestamps = bs_common::CurrentTimestamps(*graph);
  const auto current_landmark_ids = bs_common::CurrentLandmarkIDs(*graph);
  if (current_landmark_ids.empty()) {
    ROS_ERROR("Cannot use Visual Odometry without initializing with visual "
              "information.");
    throw std::runtime_error{"Cannot use Visual Odometry without "
                             "initializing with visual information."};
  }

  if (vo_params_.use_standalone_vo) {
    local_graph_ = std::move(graph->clone());
    visual_map_->UpdateGraph(*local_graph_);
  } else {
    visual_map_->UpdateGraph(*graph);
  }

  T_WORLD_BASELINKprevframe_ =
      visual_map_->GetBaselinkPose(*timestamps.rbegin()).value();

  // get measurments as a vector of timestamps
  std::vector<uint64_t> measurement_stamps;
  std::for_each(visual_measurement_buffer_.begin(),
                visual_measurement_buffer_.end(), [&](const auto& msg) {
                  measurement_stamps.push_back(msg->header.stamp.toNSec());
                });

  // get timestamps in graph as a vector
  std::vector<uint64_t> graph_stamps;
  std::for_each(timestamps.begin(), timestamps.end(), [&](const auto& stamp) {
    graph_stamps.push_back(stamp.toNSec());
  });

  // find the union between the two
  std::vector<uint64_t> union_stamps;
  std::set_intersection(graph_stamps.begin(), graph_stamps.end(),
                        measurement_stamps.begin(), measurement_stamps.end(),
                        std::inserter(union_stamps, union_stamps.begin()));

  // create a map to access measurements based on stamp
  std::map<uint64_t, bs_common::CameraMeasurementMsg::ConstPtr> measurement_map;
  std::for_each(
      visual_measurement_buffer_.begin(), visual_measurement_buffer_.end(),
      [&](auto msg) { measurement_map[msg->header.stamp.toNSec()] = msg; });

  // add each measurement as a keyframe if its in the graph
  for (const auto& stamp : union_stamps) {
    const auto msg = measurement_map.at(stamp);
    vision::Keyframe kf(*msg);
    keyframes_.insert({msg->header.stamp, kf});
    previous_keyframe_ = msg->header.stamp;
  }

  // remove measurements
  const uint64_t last_stamp = *union_stamps.rbegin();
  while (!visual_measurement_buffer_.empty() &&
         visual_measurement_buffer_.front()->header.stamp.toNSec() <
             last_stamp) {
    visual_measurement_buffer_.pop_front();
  }

  // process visual information in buffer that isn't in the graph yet
  while (!visual_measurement_buffer_.empty()) {
    const auto msg = visual_measurement_buffer_.front();
    if (!ComputeOdometryAndExtendMap(msg)) { break; }
    visual_measurement_buffer_.pop_front();
  }
}

void VisualOdometry::ProcessLandmarkIDP(
    const uint64_t id, const ros::Time& timestamp,
    fuse_core::Transaction::SharedPtr transaction) {
  auto lm = visual_map_->GetInverseDepthLandmark(id);
  if (lm) {
    // if the landmark exists, just add a constraint to the current keyframe
    try {
      Eigen::Vector2d pixel = landmark_container_->GetValue(timestamp, id);
      visual_map_->AddInverseDepthVisualConstraint(timestamp, id, pixel,
                                                   transaction);
    } catch (const std::out_of_range& oor) { return; }
  } else {
    // if the landmark doesnt exist we try to initialize it
    // triangulate and add landmark
    Eigen::Vector3d avg_viewing_angle;
    uint64_t word_id;
    const auto initial_point =
        TriangulateLandmark(id, avg_viewing_angle, word_id);
    if (!initial_point.has_value()) { return; }

    // use the first keyframe that sees the landmark as the anchor frame
    auto track = landmark_container_->GetTrack(id);
    beam_containers::LandmarkMeasurement anchor_measurement;
    for (auto m : track) {
      if (keyframes_.find(m.time_point) != keyframes_.end()) {
        anchor_measurement = m;
        break;
      }
    }

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

    // add landmark to transaction
    visual_map_->AddInverseDepthLandmark(
        bearing, inverse_depth, id, anchor_measurement.time_point, transaction);

    // add constraints to keyframes that view it
    for (const auto& [kf_stamp, kf] : keyframes_) {
      try {
        Eigen::Vector2d pixel = landmark_container_->GetValue(kf_stamp, id);
        visual_map_->AddInverseDepthVisualConstraint(timestamp, id, pixel,
                                                     transaction);
      } catch (const std::out_of_range& oor) { continue; }
    }
  }
}

void VisualOdometry::ProcessLandmarkEUC(
    const uint64_t id, const ros::Time& timestamp,
    fuse_core::Transaction::SharedPtr transaction) {
  if (visual_map_->GetLandmark(id)) {
    try {
      Eigen::Vector2d pixel = landmark_container_->GetValue(timestamp, id);
      visual_map_->AddVisualConstraint(timestamp, id, pixel, transaction);
    } catch (const std::out_of_range& oor) { return; }
  } else if (new_to_old_lm_ids_.left.find(id) !=
             new_to_old_lm_ids_.left.end()) {
    try {
      Eigen::Vector2d pixel = landmark_container_->GetValue(timestamp, id);
      visual_map_->AddVisualConstraint(
          timestamp, new_to_old_lm_ids_.left.at(id), pixel, transaction);
    } catch (const std::out_of_range& oor) { return; }
  } else {
    // triangulate landmark
    Eigen::Vector3d avg_viewing_angle;
    uint64_t word_id;
    const auto initial_point =
        TriangulateLandmark(id, avg_viewing_angle, word_id);
    if (!initial_point.has_value()) { return; }

    if (vo_params_.local_map_matching) {
      Eigen::Vector2d cur_pixel = landmark_container_->GetValue(timestamp, id);
      uint64_t matched_id;
      if (SearchLocalMap(cur_pixel, avg_viewing_angle, word_id, matched_id)) {
        // add constraint to matched id
        visual_map_->AddVisualConstraint(timestamp, matched_id, cur_pixel,
                                         transaction);
        new_to_old_lm_ids_.insert({id, matched_id});
        return;
      }
    }

    visual_map_->AddLandmark(initial_point.value(), avg_viewing_angle, word_id,
                             id, transaction);

    // add constraints to keyframes that view its
    for (const auto& [kf_stamp, kf] : keyframes_) {
      try {
        Eigen::Vector2d pixel = landmark_container_->GetValue(kf_stamp, id);
        visual_map_->AddVisualConstraint(kf_stamp, id, pixel, transaction);
      } catch (const std::out_of_range& oor) { continue; }
    }
  }
}

void VisualOdometry::UpdateLocalGraph(const fuse_core::Graph& new_graph) {
  const auto graph_stamps = bs_common::CurrentTimestamps(new_graph);

  // get the pose of a reference keyframe wrt both graphs
  ros::Time reference_kf_time;
  Eigen::Matrix4d T_WORLDmain_BASELINKrefkf;
  Eigen::Matrix4d T_WORLDlocal_BASELINKrefkf;
  for (const auto& t : graph_stamps) {
    if (keyframes_.find(t) != keyframes_.end()) {
      reference_kf_time = t;
      const auto p1 = bs_common::GetPosition(new_graph, t);
      const auto o1 = bs_common::GetOrientation(new_graph, t);
      const auto p2 = bs_common::GetPosition(*local_graph_, t);
      const auto o2 = bs_common::GetOrientation(*local_graph_, t);
      if (o1 && p1 && p2 && o2) {
        bs_common::FusePoseToEigenTransform(*p1, *o1,
                                            T_WORLDmain_BASELINKrefkf);
        bs_common::FusePoseToEigenTransform(*p2, *o2,
                                            T_WORLDlocal_BASELINKrefkf);
        break;
      }
    }
  }

  // compute pose states with respect to the local graph
  for (const auto& t : graph_stamps) {
    if (keyframes_.find(t) == keyframes_.end()) {
      const auto p = bs_common::GetPosition(new_graph, t);
      const auto o = bs_common::GetOrientation(new_graph, t);
      if (o && p) {
        // compute pose wrt local graph
        Eigen::Matrix4d T_WORLDmain_BASELINKcur;
        bs_common::FusePoseToEigenTransform(*p, *o, T_WORLDmain_BASELINKcur);
        Eigen::Matrix4d T_BASELINKrefkf_BASELINKcur =
            beam::InvertTransform(T_WORLDmain_BASELINKrefkf) *
            T_WORLDmain_BASELINKcur;
        Eigen::Matrix4d T_WORLDlocal_BASELINKcur =
            T_WORLDlocal_BASELINKrefkf * T_BASELINKrefkf_BASELINKcur;

        // add variables to local graph
        fuse_variables::Position3DStamped::SharedPtr p_local =
            std::make_shared<fuse_variables::Position3DStamped>(t);
        fuse_variables::Orientation3DStamped::SharedPtr o_local =
            std::make_shared<fuse_variables::Orientation3DStamped>(t);
        bs_common::EigenTransformToFusePose(T_WORLDlocal_BASELINKcur, *p_local,
                                            *o_local);
        local_graph_->addVariable(p_local);
        local_graph_->addVariable(o_local);
      }
    }
  }

  // add variables
  for (const auto& v : new_graph.getVariables()) {
    if (v.type() != "fuse_variables::Position3DStamped" ||
        v.type() != "fuse_variables::Orientation3DStamped") {
      local_graph_->addVariable(std::move(v.clone()));
    }
  }

  // add inertial and motion model constraints
  for (const auto& c : new_graph.getConstraints()) {
    if (c.source() == "bs_models::Unicycle3D" ||
        c.source() == "bs_models::InertialOdometry") {
      local_graph_->addConstraint(std::move(c.clone()));
    }
  }
  local_graph_->optimizeFor(ros::Duration(0.02), local_solver_options_);
}

void VisualOdometry::MarginalizeLocalGraph(const fuse_core::Graph& new_graph) {
  const auto new_timestamps = bs_common::CurrentTimestamps(new_graph);
  const auto current_timestamps = bs_common::CurrentTimestamps(*local_graph_);
  const ros::Time oldest_new_time = *new_timestamps.begin();

  for (const auto& t : current_timestamps) {
    if (t > oldest_new_time) { break; }

    std::vector<fuse_core::UUID> constraints_to_remove;

    auto p = bs_common::GetPosition(*local_graph_, t);
    if (p) {
      auto p_constraints = local_graph_->getConnectedConstraints(p->uuid());
      for (const auto& c : p_constraints) {
        constraints_to_remove.push_back(c.uuid());
      }
    }

    auto o = bs_common::GetOrientation(*local_graph_, t);
    if (o) {
      auto o_constraints = local_graph_->getConnectedConstraints(o->uuid());
      for (const auto& c : o_constraints) {
        constraints_to_remove.push_back(c.uuid());
      }
    }

    auto bg = bs_common::GetGyroscopeBias(*local_graph_, t);
    if (bg) {
      auto bg_constraints = local_graph_->getConnectedConstraints(bg->uuid());
      for (const auto& c : bg_constraints) {
        constraints_to_remove.push_back(c.uuid());
      }
    }

    auto ba = bs_common::GetAccelBias(*local_graph_, t);
    if (ba) {
      auto ba_constraints = local_graph_->getConnectedConstraints(ba->uuid());
      for (const auto& c : ba_constraints) {
        constraints_to_remove.push_back(c.uuid());
      }
    }

    auto v = bs_common::GetVelocity(*local_graph_, t);
    if (v) {
      auto v_constraints = local_graph_->getConnectedConstraints(v->uuid());
      for (const auto& c : v_constraints) {
        constraints_to_remove.push_back(c.uuid());
      }
    }

    for (const auto uuid : constraints_to_remove) {
      if (local_graph_->constraintExists(uuid)) {
        try {
          local_graph_->removeConstraint(uuid);
        } catch (const std::exception& e) {}
      }
    }

    if (p) { local_graph_->removeVariable(p->uuid()); }
    if (o) { local_graph_->removeVariable(o->uuid()); }
    if (ba) { local_graph_->removeVariable(ba->uuid()); }
    if (bg) { local_graph_->removeVariable(bg->uuid()); }
    if (v) { local_graph_->removeVariable(v->uuid()); }
  }

  // remove any disconnected landmarks
  const auto landmarks = bs_common::CurrentLandmarkIDs(*local_graph_);
  for (auto& lm : landmarks) {
    const auto lm_uuid = visual_map_->GetLandmarkUUID(lm);
    auto constraints = local_graph_->getConnectedConstraints(lm_uuid);
    auto num_constraints =
        std::distance(constraints.begin(), constraints.end());
    if (constraints.empty()) { local_graph_->removeVariable(lm_uuid); }
  }
}

fuse_core::Transaction::SharedPtr VisualOdometry::CreateVisualOdometryFactor(
    const ros::Time& timestamp_curframe,
    const Eigen::Matrix4d& T_WORLD_BASELINKcurframe,
    const Eigen::Matrix<double, 6, 6>& covariance) {
  // retrieve previous keyframe pose from the main graph
  Eigen::Matrix4d T_WORLDmain_BASELINKprevkf;
  if (!frame_initializer_->GetPose(T_WORLDmain_BASELINKprevkf,
                                   previous_keyframe_,
                                   extrinsics_.GetBaselinkFrameId())) {
    ROS_FATAL("Cannot retrieve previous keyframe pose.");
    throw std::runtime_error{"Cannot retrieve previous keyframe pose."};
  }

  // retrieve previous keyframe pose from the local graph
  const auto T_WORLD_BASELINKprevkf =
      visual_map_->GetBaselinkPose(previous_keyframe_);
  if (!T_WORLD_BASELINKprevkf.has_value()) {
    ROS_FATAL("Cannot retrieve previous keyframe pose.");
    throw std::runtime_error{"Cannot retrieve previous keyframe pose."};
  }

  // compute relative motion between this frame and previous keyframe in the
  // local graph
  const auto T_prevkf_curframe =
      beam::InvertTransform(T_WORLD_BASELINKprevkf.value()) *
      T_WORLD_BASELINKcurframe;

  // compute pose of this frame wrt the main graph
  const Eigen::Matrix4d T_WORLDmain_BASELINKcurframe =
      T_WORLDmain_BASELINKprevkf * T_prevkf_curframe;

  // send a relative pose constraint to the main graph
  auto pose_transaction = fuse_core::Transaction::make_shared();
  pose_transaction->stamp(timestamp_curframe);

  // add pose to transaction
  Eigen::Quaterniond q;
  Eigen::Vector3d p;
  beam::TransformMatrixToQuaternionAndTranslation(T_WORLDmain_BASELINKcurframe,
                                                  q, p);
  fuse_variables::Orientation3DStamped::SharedPtr orientation =
      fuse_variables::Orientation3DStamped::make_shared(timestamp_curframe);
  orientation->w() = q.w();
  orientation->x() = q.x();
  orientation->y() = q.y();
  orientation->z() = q.z();
  fuse_variables::Position3DStamped::SharedPtr position =
      fuse_variables::Position3DStamped::make_shared(timestamp_curframe);
  position->x() = p[0];
  position->y() = p[1];
  position->z() = p[2];
  pose_transaction->addVariable(position);
  pose_transaction->addVariable(orientation);

  const Eigen::Matrix<double, 6, 6> weighted_cov =
      vo_params_.odom_covariance_weight * covariance;
  if (!vo_params_.use_online_calibration) {
    const fuse_core::Vector7d delta_prevkf_curframe =
        bs_common::ComputeDelta(T_prevkf_curframe);
    visual_map_->AddRelativePoseConstraint(
        previous_keyframe_, timestamp_curframe, delta_prevkf_curframe,
        weighted_cov, pose_transaction);
  } else {
    // compute the delta wrt camera pose not baselink
    const Eigen::Matrix4d T_CAMprevkf_WORLD =
        T_cam_baselink_ * beam::InvertTransform(T_WORLD_BASELINKprevkf.value());
    const Eigen::Matrix4d T_WORLD_CAMcurframe =
        T_WORLD_BASELINKcurframe * T_baselink_cam_;
    const Eigen::Matrix4d T_CAMprevkf_CAMcurframe =
        T_CAMprevkf_WORLD * T_WORLD_CAMcurframe;
    const fuse_core::Vector7d delta_prevkf_curframe =
        bs_common::ComputeDelta(T_CAMprevkf_CAMcurframe);
    visual_map_->AddRelativePoseConstraintOnlineCalib(
        previous_keyframe_, timestamp_curframe, delta_prevkf_curframe,
        weighted_cov, pose_transaction);
  }
  return pose_transaction;
}

bool VisualOdometry::GetInitialPoseEstimate(const ros::Time& timestamp,
                                            Eigen::Matrix4d& T_WORLD_BASELINK) {
  std::string error;
  if (use_frame_init_relative_) {
    Eigen::Matrix4d T_PREVKF_CURFRAME;
    if (!frame_initializer_->GetRelativePose(T_PREVKF_CURFRAME,
                                             previous_keyframe_, timestamp)) {
      ROS_WARN_STREAM(
          "Unable to retrieve relative pose from last keyframe: " << timestamp);
      return false;
    }
    auto T_WORLD_BASELINKprevkf =
        visual_map_->GetBaselinkPose(previous_keyframe_);
    if (!T_WORLD_BASELINKprevkf.has_value()) {
      ROS_FATAL("Cannot retrieve previous keyframe pose.");
      throw std::runtime_error{"Cannot retrieve previous keyframe pose."};
    }
    T_WORLD_BASELINK = T_WORLD_BASELINKprevkf.value() * T_PREVKF_CURFRAME;
  } else {
    if (!frame_initializer_->GetPose(T_WORLD_BASELINK, timestamp,
                                     extrinsics_.GetBaselinkFrameId(), error)) {
      ROS_WARN_STREAM("Unable to estimate pose from frame initializer, "
                      "buffering frame: "
                      << timestamp << ".\n\tError: " << error);
      return false;
    }
  }
  return true;
}

void VisualOdometry::PruneKeyframes(const fuse_core::Graph& new_graph) {
  // publish keyframes as chunks and remove them
  const auto new_timestamps = bs_common::CurrentTimestamps(new_graph);
  while (!keyframes_.empty() &&
         new_timestamps.find((*keyframes_.begin()).first) ==
             new_timestamps.end()) {
    PublishSlamChunk((*keyframes_.begin()).second);
    keyframes_.erase((*keyframes_.begin()).first);
  }
}

void VisualOdometry::PublishLandmarkPointCloud(const fuse_core::Graph& graph) {
  static size_t count = 0;
  // publish landmarks as a point cloud
  pcl::PointCloud<pcl::PointXYZRGBL> cloud =
      bs_models::graph_visualization::GetGraphCameraLandmarksAsCloud(graph);
  sensor_msgs::PointCloud2 ros_cloud = beam::PCLToROS<pcl::PointXYZRGBL>(
      cloud, ros::Time::now(), extrinsics_.GetWorldFrameId(), count++);
  camera_landmarks_publisher_.publish(ros_cloud);
}

void VisualOdometry::PublishOdometry(
    const ros::Time& timestamp, const Eigen::Matrix4d& T_WORLD_BASELINK,
    const Eigen::Matrix<double, 6, 6>& covariance) {
  static uint64_t rel_odom_seq = 0;
  // publish to odometry topic
  const auto odom_msg = bs_common::TransformToOdometryMessage(
      timestamp, rel_odom_seq++, extrinsics_.GetWorldFrameId(),
      extrinsics_.GetBaselinkFrameId(), T_WORLD_BASELINK, covariance);
  odometry_publisher_.publish(odom_msg);
}

void VisualOdometry::PublishSlamChunk(const vision::Keyframe& keyframe) {
  static uint64_t slam_chunk_seq = 0;
  const Eigen::Matrix4d T_WORLD_BASELINK =
      visual_map_->GetBaselinkPose(keyframe.Stamp()).value();
  bs_common::SlamChunkMsg slam_chunk_msg;
  geometry_msgs::PoseStamped pose_stamped;
  bs_common::EigenTransformToPoseStamped(
      T_WORLD_BASELINK, keyframe.Stamp(), slam_chunk_seq++,
      extrinsics_.GetBaselinkFrameId(), pose_stamped);
  slam_chunk_msg.T_WORLD_BASELINK = pose_stamped;
  slam_chunk_msg.camera_measurement = keyframe.MeasurementMessage();

  nav_msgs::Path sub_keyframe_path;
  sub_keyframe_path.header.stamp = keyframe.Stamp();
  sub_keyframe_path.header.frame_id = extrinsics_.GetBaselinkFrameId();
  for (const auto [stamp, pose] : keyframe.Trajectory()) {
    geometry_msgs::PoseStamped pose_stamped;
    bs_common::EigenTransformToPoseStamped(
        pose, stamp, 0, extrinsics_.GetBaselinkFrameId(), pose_stamped);
    sub_keyframe_path.poses.push_back(pose_stamped);
  }

  slam_chunk_msg.trajectory_measurement = sub_keyframe_path;
  slam_chunk_publisher_.publish(slam_chunk_msg);
}

void VisualOdometry::PublishPose(const ros::Time& timestamp,
                                 const Eigen::Matrix4d& T_WORLD_BASELINK) {
  static uint64_t kf_odom_seq = 0;
  geometry_msgs::PoseStamped msg;
  bs_common::EigenTransformToPoseStamped(T_WORLD_BASELINK, timestamp,
                                         kf_odom_seq++,
                                         extrinsics_.GetBaselinkFrameId(), msg);
  keyframe_publisher_.publish(msg);
}

void VisualOdometry::ProjectMapPoints(const Eigen::Matrix4d& T_WORLD_BASELINK) {
  landmark_projection_mask_ =
      cv::Mat::zeros(cam_model_->GetWidth(), cam_model_->GetHeight(), CV_8UC1);
  image_projection_to_lm_id_.clear();

  const auto landmarks = visual_map_->GetLandmarks();
  for (const auto& [id, point] : landmarks) {
    // transform into current frame
    Eigen::Vector3d point_t_cam =
        ((T_cam_baselink_ * beam::InvertTransform(T_WORLD_BASELINK)) *
         point.homogeneous())
            .hnormalized();
    // project into image and store
    Eigen::Vector2d pixel;
    bool in_image = false;
    if (!cam_model_->ProjectPoint(point_t_cam, pixel, in_image) || !in_image) {
      continue;
    }
    landmark_projection_mask_.at<uchar>(pixel[0], pixel[1]) = 1;
    uint64_t pixel_index = pixel[1] * cam_model_->GetWidth() + pixel[0];
    image_projection_to_lm_id_[pixel_index] = id;
  }
}

bool VisualOdometry::SearchLocalMap(const Eigen::Vector2d& pixel,
                                    const Eigen::Vector3d& viewing_angle,
                                    const uint64_t word_id,
                                    uint64_t& matched_id) {
  // compute the search radius around the pixel
  Eigen::Vector2i top_left{static_cast<int>(pixel[0]) - 10,
                           static_cast<int>(pixel[1]) - 10};
  Eigen::Vector2i bottom_right{static_cast<int>(pixel[0]) + 10,
                               static_cast<int>(pixel[1]) + 10};
  if (top_left[0] < 0) { top_left[0] = 0; }
  if (top_left[1] < 0) { top_left[1] = 0; }
  if (bottom_right[0] > cam_model_->GetHeight()) {
    bottom_right[0] = cam_model_->GetHeight();
  }
  if (bottom_right[1] > cam_model_->GetWidth()) {
    bottom_right[1] = cam_model_->GetWidth();
  }

  // find all candidate landmarks within the radius
  std::vector<uint64_t> candidate_lm_matches;
  for (int row = top_left[1]; row < bottom_right[1]; row++) {
    for (int col = top_left[0]; col < bottom_right[0]; col++) {
      if (landmark_projection_mask_.at<uchar>(col, row) != 0) {
        uint64_t pixel_index = row * cam_model_->GetWidth() + col;
        candidate_lm_matches.push_back(image_projection_to_lm_id_[pixel_index]);
      }
    }
  }

  // check each candidate for a match
  for (const uint64_t id : candidate_lm_matches) {
    auto landmark = visual_map_->GetLandmark(id);
    if (!landmark) { continue; }
    if (landmark->word_id() == word_id) {
      const double angle =
          std::acos((viewing_angle.dot(landmark->viewing_angle())) /
                    (viewing_angle.norm() * landmark->viewing_angle().norm()));
      if (beam::Rad2Deg(angle) < 10.0) {
        matched_id = id;
        return true;
      }
    }
  }
  return false;
}

void VisualOdometry::CleanNewToOldLandmarkMap(
    const std::set<uint64_t>& old_ids, const std::set<uint64_t>& new_ids) {
  // find out which landmark id's have been removed
  std::set<uint64_t> removed_ids;
  std::set_difference(old_ids.begin(), old_ids.end(), new_ids.begin(),
                      new_ids.end(),
                      std::inserter(removed_ids, removed_ids.begin()));

  // remove from the association map
  for (const uint64_t id : removed_ids) {
    if (new_to_old_lm_ids_.right.find(id) != new_to_old_lm_ids_.right.end()) {
      new_to_old_lm_ids_.right.erase(id);
    }
  }
}

double VisualOdometry::ComputeAverageReprojection(
    const Eigen::Matrix4d& T_WORLD_BASELINK,
    const std::vector<Eigen::Vector2i, beam::AlignVec2i>& pixels,
    const std::vector<Eigen::Vector3d, beam::AlignVec3d>& points) {
  if (pixels.size() == 0) { return 0; }
  double total_reproj = 0.0;
  const Eigen::Matrix4d T_BASELINK_WORLD =
      beam::InvertTransform(T_WORLD_BASELINK);
  for (int i = 0; i < pixels.size(); i++) {
    const auto p_WORLD = points[i];
    const Eigen::Vector3d p_CAMERA =
        ((T_cam_baselink_ * T_BASELINK_WORLD) * p_WORLD.homogeneous())
            .hnormalized();

    Eigen::Vector2d projection;
    bool in_image{false};
    if (!cam_model_->ProjectPoint(p_CAMERA, projection, in_image) ||
        !in_image) {
      continue;
    } else {
      total_reproj += (pixels[i].cast<double>() - projection).norm();
    }
  }

  return total_reproj / static_cast<double>(pixels.size());
}

void VisualOdometry::shutdown() {
  measurement_subscriber_.shutdown();
  imu_constraint_trigger_counter_ = 0;
  num_loc_fails_in_a_row_ = 0;
  track_lost_ = false;
  image_projection_to_lm_id_.clear();
  new_to_old_lm_ids_.clear();
  is_initialized_ = false;
  keyframes_.clear();
  visual_measurement_buffer_.clear();
  // frame_initializer_->Clear();
  prev_frame_ = ros::Time(0);
  local_graph_->clear();
  visual_map_->Clear();
  validator_->Clear();
  image_db_->Clear();
}

} // namespace bs_models
