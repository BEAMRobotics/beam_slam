#include <bs_models/visual_odometry.h>

#include <bs_constraints/visual/euclidean_reprojection_constraint.h>
#include <fuse_core/transaction.h>
#include <fuse_variables/acceleration_linear_3d_stamped.h>
#include <fuse_variables/velocity_angular_3d_stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Time.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/Utils.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <bs_common/conversions.h>
#include <bs_common/graph_access.h>

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

  // create frame initializer (inertial, lidar, constant velocity)
  frame_initializer_ = std::make_unique<bs_models::FrameInitializer>(
      vo_params_.frame_initializer_config);

  // Load camera model and create visua map object
  cam_model_ = beam_calibration::CameraModel::Create(
      calibration_params_.cam_intrinsics_path);
  cam_intrinsic_matrix_ = cam_model_->GetRectifiedModel()->GetIntrinsicMatrix();
  visual_map_ =
      std::make_shared<VisualMap>(cam_model_, vo_params_.reprojection_loss,
                                  vo_params_.reprojection_information_weight);

  // Initialize landmark measurement container
  landmark_container_ = std::make_shared<beam_containers::LandmarkContainer>();

  // create pose refiner for motion only BA
  pose_refiner_ = std::make_shared<beam_cv::PoseRefinement>(0.02, true, 0.2);

  // get extrinsics
  extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_);

  // compute the max container size
  ros::param::get("/local_mapper/lag_duration", lag_duration_);
  max_container_size_ = calibration_params_.camera_hz * (lag_duration_ + 1);
}

void VisualOdometry::onStart() {
  // setup subscribers
  measurement_subscriber_ =
      private_node_handle_.subscribe<bs_common::CameraMeasurementMsg>(
          ros::names::resolve(
              "/local_mapper/visual_feature_tracker/visual_measurements"),
          10, &ThrottledMeasurementCallback::callback,
          &throttled_measurement_callback_,
          ros::TransportHints().tcpNoDelay(false));

  // setup publishers
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
  reloc_publisher_ = private_node_handle_.advertise<bs_common::RelocRequestMsg>(
      "/local_mapper/reloc_request", 100);
}

void VisualOdometry::processMeasurements(
    const bs_common::CameraMeasurementMsg::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE(
      "VisualOdometry received VISUAL measurements: " << msg->header.stamp);

  // add measurements to local container
  AddMeasurementsToContainer(msg);

  // buffer the message
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
  if (!LocalizeFrame(timestamp, T_WORLD_BASELINK)) { return false; }

  // compute and publish relative odometry
  ComputeRelativeOdometry(timestamp, T_WORLD_BASELINK);

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
  ROS_INFO_STREAM("VisualOdometry: New keyframe detected at: " << timestamp);
  Keyframe kf(*msg);
  keyframes_.insert({timestamp, kf});
  previous_keyframe_ = timestamp;

  // extend existing map and add constraints
  ExtendMap(timestamp, T_WORLD_BASELINK);

  // publish keyframe pose
  PublishPose(timestamp, T_WORLD_BASELINK);

  // send IO trigger
  if (vo_params_.trigger_inertial_odom_constraints) {
    std_msgs::Time time_msg;
    time_msg.data = timestamp;
    imu_constraint_trigger_publisher_.publish(time_msg);
    imu_constraint_trigger_counter_++;
  }

  // publish reloc request at given rate
  if ((timestamp - previous_reloc_request_).toSec() >
      vo_params_.reloc_request_period) {
    ROS_INFO_STREAM("Publishing reloc request at: " << timestamp);
    previous_reloc_request_ = timestamp;
    PublishRelocRequest(kf);
  }

  return true;
}

bool VisualOdometry::LocalizeFrame(const ros::Time& timestamp,
                                   Eigen::Matrix4d& T_WORLD_BASELINK) {
  std::string error;
  Eigen::Matrix4d T_WORLD_BASELINKcur;
  if (!frame_initializer_->GetPose(T_WORLD_BASELINKcur, timestamp,
                                   extrinsics_.GetBaselinkFrameId(), error)) {
    ROS_WARN_STREAM("Unable to estimate pose from frame initializer, "
                    "buffering frame: "
                    << timestamp << ".\n\tError: " << error);
    return false;
  }

  // get 2d-3d correspondences
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  GetPixelPointPairs(timestamp, pixels, points);

  // perform motion only BA to refine estimate
  if (pixels.size() >= 20) {
    if (track_lost) { track_lost = false; }
    // get initial estimate in camera frame
    Eigen::Matrix4d T_CAMERA_WORLD_est = beam::InvertTransform(
        T_WORLD_BASELINKcur * beam::InvertTransform(T_cam_baselink_));
    // add a prior on the initial imu estimate
    Eigen::Matrix<double, 6, 6> prior =
        1e-5 * Eigen::Matrix<double, 6, 6>::Identity();
    // perform non-linear pose refinement
    Eigen::Matrix4d T_CAMERA_WORLD_ref = pose_refiner_->RefinePose(
        T_CAMERA_WORLD_est, cam_model_, pixels, points,
        std::make_shared<Eigen::Matrix<double, 6, 6>>(prior));
    T_WORLD_BASELINK =
        beam::InvertTransform(T_CAMERA_WORLD_ref) * T_cam_baselink_;

    // output the difference between the imu and vo estimates
    Eigen::Vector3d diff = T_WORLD_BASELINKcur.block<3, 1>(0, 3) -
                           T_WORLD_BASELINK.block<3, 1>(0, 3);
    ROS_DEBUG_STREAM("Difference between IMU estimate and VO estimate: ["
                     << diff.x() << ", " << diff.y() << ", " << diff.z()
                     << "]");
  } else {
    ROS_WARN_STREAM(
        "Not enough points for visual refinement: " << pixels.size());
    T_WORLD_BASELINK = T_WORLD_BASELINKcur;
    track_lost = true;
  }

  return true;
}

void VisualOdometry::ExtendMap(const ros::Time& timestamp,
                               const Eigen::Matrix4d& T_WORLD_BASELINK) {
  // create transaction for this keyframe
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(timestamp);
  visual_map_->AddBaselinkPose(T_WORLD_BASELINK, timestamp, transaction);

  // add prior if using a frame initializer
  if (frame_initializer_ && vo_params_.prior_information_weight != 0) {
    visual_map_->AddPosePrior(timestamp, vo_params_.prior_covariance,
                              transaction);
  }

  // process each landmark
  const auto landmarks = landmark_container_->GetLandmarkIDsInImage(timestamp);
  for (const auto id : landmarks) {
    if (vo_params_.use_idp) {
      ProcessLandmarkIDP(id, timestamp, transaction);
    } else {
      ProcessLandmarkEUC(id, timestamp, transaction);
    }
  }

  sendTransaction(transaction);
}

void VisualOdometry::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  ROS_INFO_STREAM_ONCE("VisualOdometry received initial graph.");

  // publish marginalized keyframes as slam chunks (if we have initialized)
  if (is_initialized_) {
    // all timestamps in the new graph
    const auto timestamps = bs_common::CurrentTimestamps(graph);
    while (!keyframes_.empty() &&
           timestamps.find((*keyframes_.begin()).first) == timestamps.end()) {
      PublishSlamChunk((*keyframes_.begin()).second);
      keyframes_.erase((*keyframes_.begin()).first);
    }
  }

  // Update graph object in visual map
  visual_map_->UpdateGraph(graph);

  // do initial setup
  if (!is_initialized_) { Initialize(graph); }
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

  const auto kf_time = (*keyframes_.rbegin()).first;
  Eigen::Matrix4d T_PREVKF_CURFRAME;
  if (!frame_initializer_->GetRelativePose(T_PREVKF_CURFRAME, kf_time,
                                           timestamp)) {
    ROS_WARN_STREAM(
        "Unable to retrieve relative pose from last keyframe: " << timestamp);
    return false;
  }

  // compute rotation adjusted parallax
  Eigen::Matrix3d R_PREVKF_CURFRAME = T_PREVKF_CURFRAME.block<3, 3>(0, 0);
  std::vector<uint64_t> frame1_ids =
      landmark_container_->GetLandmarkIDsInImage(kf_time);
  double total_parallax = 0.0;
  int num_correspondences = 0;
  std::vector<double> parallaxes;
  for (auto& id : frame1_ids) {
    try {
      Eigen::Vector2d p1 = landmark_container_->GetValue(kf_time, id);
      Eigen::Vector2d p2 = landmark_container_->GetValue(timestamp, id);
      Eigen::Vector3d bp2;
      if (!cam_model_->BackProject(p1.cast<int>(), bp2)) { continue; }
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
  } else if ((timestamp - kf_time).toSec() > ((lag_duration_ / 2.0) - 0.5)) {
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
    cv::Mat K(3, 3, CV_32F);
    cv::eigen2cv(cam_intrinsic_matrix_, K);
    std::vector<uchar> mask;
    cv::findEssentialMat(fp1, fp2, K, cv::RANSAC, 0.99, 2.0, mask);

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
    VisualOdometry::TriangulateLandmark(const uint64_t id) {
  std::vector<Eigen::Matrix4d, beam::AlignMat4d> T_cam_world_v;
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  beam_containers::Track track = landmark_container_->GetTrack(id);
  for (auto& m : track) {
    const auto T_camera_world = visual_map_->GetCameraPose(m.time_point);
    // check if the pose is in the graph
    if (T_camera_world.has_value()) {
      pixels.push_back(m.value.cast<int>());
      T_cam_world_v.push_back(beam::InvertTransform(T_camera_world.value()));
    }
  }
  // must have at least 3 keyframes that have seen the landmark
  if (T_cam_world_v.size() >= 2) {
    // if we've lost track, ease the requirements on new landmarks
    if (track_lost) {
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
  for (auto& id : landmarks) {
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
      bs_variables::Point3DLandmark::SharedPtr lm =
          visual_map_->GetLandmark(id);
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

void VisualOdometry::ComputeRelativeOdometry(
    const ros::Time& timestamp, const Eigen::Matrix4d& T_WORLD_BASELINKcur) {
  static uint64_t rel_odom_seq = 0;
  // todo: fix this, the odom pose is garbage, the normal pose is fine
  // const auto prev_kf_pose = visual_map_->GetBaselinkPose(previous_keyframe_);
  // const Eigen::Matrix4d T_WORLD_BASELINKprev = prev_kf_pose.value();
  // const Eigen::Matrix4d T_PREVKF_CURFRAME =
  //     beam::InvertTransform(T_WORLD_BASELINKprev) * T_WORLD_BASELINKcur;
  // const Eigen::Matrix4d T_ODOM_BASELINKcur =
  //     T_ODOM_BASELINKprev_ * T_PREVKF_CURFRAME;
  // // update odom pose
  // T_ODOM_BASELINKprev_ = T_ODOM_BASELINKcur;

  // publish to odometry topic
  const auto odom_msg = bs_common::TransformToOdometryMessage(
      timestamp, rel_odom_seq++, extrinsics_.GetWorldFrameId(),
      extrinsics_.GetBaselinkFrameId(), T_WORLD_BASELINKcur);
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
  slam_chunk_publisher_.publish(slam_chunk_msg);
  // todo: get sub trajectory and publish
}

void VisualOdometry::PublishRelocRequest(const vision::Keyframe& keyframe) {
  static uint64_t reloc_seq = 0;
  const Eigen::Matrix4d T_WORLD_BASELINK =
      visual_map_->GetBaselinkPose(keyframe.Stamp()).value();
  bs_common::RelocRequestMsg reloc_msg;
  geometry_msgs::PoseStamped pose_stamped;
  bs_common::EigenTransformToPoseStamped(
      T_WORLD_BASELINK, keyframe.Stamp(), reloc_seq++,
      extrinsics_.GetBaselinkFrameId(), pose_stamped);
  reloc_msg.T_WORLD_BASELINK = pose_stamped;
  reloc_msg.camera_measurement = keyframe.MeasurementMessage();
  reloc_publisher_.publish(reloc_msg);
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

void VisualOdometry::Initialize(fuse_core::Graph::ConstSharedPtr graph) {
  const auto timestamps = bs_common::CurrentTimestamps(graph);
  const auto current_landmark_ids = bs_common::CurrentLandmarkIDs(graph);
  if (current_landmark_ids.empty()) {
    ROS_ERROR("Cannot use Visual Odometry without initializing with visual "
              "information.");
    throw std::runtime_error{"Cannot use Visual Odometry without "
                             "initializing with visual information."};
  }

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
    T_ODOM_BASELINKprev_ =
        visual_map_->GetBaselinkPose(msg->header.stamp).value();
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

  is_initialized_ = true;
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
    const auto initial_point = TriangulateLandmark(id);
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
  } else {
    // triangulate and add landmark
    const auto initial_point = TriangulateLandmark(id);
    if (!initial_point.has_value()) { return; }
    visual_map_->AddLandmark(initial_point.value(), id, transaction);

    // add constraints to keyframes that view its
    for (const auto& [kf_stamp, kf] : keyframes_) {
      try {
        Eigen::Vector2d pixel = landmark_container_->GetValue(kf_stamp, id);
        visual_map_->AddVisualConstraint(kf_stamp, id, pixel, transaction);
      } catch (const std::out_of_range& oor) { continue; }
    }
  }
}

} // namespace bs_models
