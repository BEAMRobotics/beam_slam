#include <bs_models/visual_odometry.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt64MultiArray.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/Utils.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <bs_common/utils.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::VisualOdometry, fuse_core::SensorModel)

namespace bs_models {

using namespace vision;

VisualOdometry::VisualOdometry()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_measurement_callback_(std::bind(
          &VisualOdometry::processMeasurements, this, std::placeholders::_1)) {}

void VisualOdometry::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  vo_params_.loadFromROS(private_node_handle_);
  calibration_params_.loadFromROS();

  // create frame initializer (inertial, lidar, constant velocity)
  frame_initializer_ =
      bs_models::frame_initializers::FrameInitializerBase::Create(
          vo_params_.frame_initializer_config);

  // Load camera model and create visua map object
  cam_model_ = beam_calibration::CameraModel::Create(
      calibration_params_.cam_intrinsics_path);
  visual_map_ = std::make_shared<VisualMap>(
      cam_model_, vo_params_.reprojection_loss,
      Eigen::Matrix2d::Identity() * vo_params_.reprojection_covariance_weight);

  // Initialize landmark measurement container
  landmark_container_ = std::make_shared<beam_containers::LandmarkContainer>();

  // create pose refiner for motion only BA
  pose_refiner_ = std::make_shared<beam_cv::PoseRefinement>(1e-1, true, 1.0);

  // get extrinsics
  extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_);
}

void VisualOdometry::onStart() {
  // setup subscribers
  measurement_subscriber_ =
      private_node_handle_.subscribe<CameraMeasurementMsg>(
          ros::names::resolve(vo_params_.visual_measurement_topic), 100,
          &ThrottledMeasurementCallback::callback,
          &throttled_measurement_callback_,
          ros::TransportHints().tcpNoDelay(false));

  // setup publishers
  odometry_publisher_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odom/relative", 100);
  keyframe_publisher_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odom/keyframes", 100);
  slam_chunk_publisher_ =
      private_node_handle_.advertise<bs_common::SlamChunkMsg>(
          "/local_mapper/slam_results", 100);
  reloc_publisher_ = private_node_handle_.advertise<bs_common::RelocRequestMsg>(
      "/local_mapper/reloc_request", 100);
}

void VisualOdometry::processMeasurements(
    const CameraMeasurementMsg::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE(
      "VisualOdometry received VISUAL measurements: " << msg->header.stamp);

  // add measurements to local container
  AddMeasurementsToContainer(msg);

  // buffer the message
  visual_measurement_buffer_.push_back(msg);

  // don't process until we have initialized
  if (!is_initialized_) { return; }

  while (!visual_measurement_buffer_.empty()) {
    // retrieve and process the message at the front of the buffer
    const auto current_msg = visual_measurement_buffer_.front();
    const auto success = ComputeOdometryAndExtendMap(current_msg);

    // ! This should only fail if the frame initializer fails, in which
    // ! case we just need to wait until it succeeds
    if (!success) { break; }

    visual_measurement_buffer_.pop_front();
  }

  // remove measurements from container if we are over the limit
  while (landmark_container_->NumImages() > vo_params_.max_container_size) {
    landmark_container_->PopFront();
  }
}

bool VisualOdometry::ComputeOdometryAndExtendMap(
    const CameraMeasurementMsg::ConstPtr& msg) {
  // odometry sequence numbers
  static uint64_t kf_odom_seq = 0;

  const auto timestamp = msg->header.stamp;
  // estimate pose of frame wrt current graph
  Eigen::Matrix4d T_WORLD_BASELINK;
  if (!LocalizeFrame(timestamp, T_WORLD_BASELINK)) { return false; }

  // compute and publish relative odometry
  ComputeRelativeOdometry(timestamp, T_WORLD_BASELINK);

  // add pose to graph
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(timestamp);
  visual_map_->AddBaselinkPose(T_WORLD_BASELINK, timestamp, transaction);
  sendTransaction(transaction);
  previous_frame_ = timestamp;

  if (IsKeyframe(timestamp, T_WORLD_BASELINK)) {
    ROS_INFO_STREAM("VisualOdometry: New keyframe detected at: " << timestamp);
    Keyframe kf(*msg);
    keyframes_.push_back(kf);
    ExtendMap(T_WORLD_BASELINK);

    // publish keyframe as odometry
    const auto kf_odom_msg = bs_common::TransformToOdometryMessage(
        kf.Stamp(), kf_odom_seq++, extrinsics_.GetWorldFrameId(),
        extrinsics_.GetBaselinkFrameId(), T_WORLD_BASELINK);
    keyframe_publisher_.publish(kf_odom_msg);

    // publish reloc request at given rate
    if ((kf.Stamp() - previous_reloc_request_).toSec() >
        vo_params_.reloc_request_period) {
      ROS_INFO_STREAM("Publishing reloc request at: " << kf.Stamp());
      previous_reloc_request_ = kf.Stamp();
      PublishRelocRequest(kf);
    }
  }

  return true;
}

bool VisualOdometry::LocalizeFrame(const ros::Time& img_time,
                                   Eigen::Matrix4d& T_WORLD_BASELINK) {
  const auto prev_frame_pose = visual_map_->GetBaselinkPose(previous_frame_);
  assert(prev_frame_pose.has_value() &&
         "Cannot retrieve previous keyframe pose.");
  const Eigen::Matrix4d T_WORLD_BASELINKprev = prev_frame_pose.value();

  // get 2d-3d correspondences
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  GetPixelPointPairs(img_time, pixels, points);
  // todo: filter outliers?

  // perform motion only BA to refine estimate
  if (pixels.size() >= 20) {
    Eigen::Matrix4d T_CAMERA_WORLD_est = beam::InvertTransform(
        T_WORLD_BASELINKprev * beam::InvertTransform(T_cam_baselink_));

    Eigen::Matrix4d T_CAMERA_WORLD_ref = pose_refiner_->RefinePose(
        T_CAMERA_WORLD_est, cam_model_, pixels, points);

    std::cout << "\n" << img_time << std::endl;
    std::cout << pixels.size() << std::endl;
    std::cout << T_WORLD_BASELINKprev << std::endl;
    T_WORLD_BASELINK =
        beam::InvertTransform(T_CAMERA_WORLD_ref) * T_cam_baselink_;
    std::cout << T_WORLD_BASELINK << std::endl;
  } else {
    // use frame initializer for pose
    Eigen::Matrix4d T_PREVFRAME_CURFRAME;
    if (!frame_initializer_->GetRelativePose(T_PREVFRAME_CURFRAME,
                                             previous_frame_, img_time)) {
      ROS_WARN_STREAM("Unable to estimate pose from frame initializer, "
                      "buffering frame: "
                      << img_time);
      return false;
    }
    T_WORLD_BASELINK = T_WORLD_BASELINKprev * T_PREVFRAME_CURFRAME;
    // todo: add constraint to imu biases
  }

  return true;
}

bool VisualOdometry::IsKeyframe(const ros::Time& img_time,
                                const Eigen::Matrix4d& T_WORLD_BASELINK) {
  if (keyframes_.empty()) { return true; }

  const auto kf_time = keyframes_.back().Stamp();
  const Eigen::Matrix4d kf_pose = visual_map_->GetBaselinkPose(kf_time).value();

  // if (vo_params_.use_parallax) {
  //   // check for parallax
  //   const auto avg_parallax =
  //       landmark_container_->ComputeParallax(kf_time, img_time, false);
  //   if (avg_parallax > vo_params_.keyframe_parallax) { return true; }
  // } else {
  //   // check for movement
  //   const auto passed_motion = beam::PassedMotionThreshold(
  //       T_WORLD_BASELINK, kf_pose, vo_params_.keyframe_rotation_deg,
  //       vo_params_.keyframe_translation_m, true);
  //   if (passed_motion) { return true; }
  // }

  // check for max duration in case of no motion
  if ((img_time - kf_time).toSec() > vo_params_.keyframe_max_duration) {
    return true;
  }

  return false;
}

void VisualOdometry::ExtendMap(const Eigen::Matrix4d& T_WORLD_BASELINK) {
  // get current and previous keyframe timestamp
  const auto prev_kf_time = (keyframes_[keyframes_.size() - 2]).Stamp();
  const auto cur_kf_time = keyframes_.back().Stamp();

  // create transaction for this keyframe
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(cur_kf_time);

  // add pose to map
  // visual_map_->AddBaselinkPose(T_WORLD_BASELINK, cur_kf_time, transaction);
  // TODO: add angular velocity and linear acceleration variables

  // add prior if using a frame initializer
  if (frame_initializer_ && vo_params_.use_pose_priors) {
    auto prior = MakeFrameInitPrior(cur_kf_time, vo_params_.prior_covariance);
    transaction->addConstraint(prior);
  }

  // process each landmark
  const auto landmarks =
      landmark_container_->GetLandmarkIDsInImage(cur_kf_time);
  auto process_landmark = [&](const auto& id) {
    if (visual_map_->GetLandmark(id)) {
      Eigen::Vector2d pixel;
      try {
        pixel = landmark_container_->GetValue(cur_kf_time, id);
      } catch (const std::out_of_range& oor) { return; }
      // add constraint
      visual_map_->AddVisualConstraint(cur_kf_time, id, pixel, transaction);
    } else {
      // triangulate and add landmark
      const auto initial_point = TriangulateLandmark(id);
      if (!initial_point.has_value()) { return; }
      visual_map_->AddLandmark(initial_point.value(), id, transaction);

      // add constraints to keyframes that view its
      for (const auto& kf : keyframes_) {
        const auto stamp = kf.Stamp();
        Eigen::Vector2d pixel;
        try {
          pixel = landmark_container_->GetValue(stamp, id);
        } catch (const std::out_of_range& oor) { continue; }
        visual_map_->AddVisualConstraint(stamp, id, pixel, transaction);
      }
    }
  };
  std::for_each(landmarks.begin(), landmarks.end(), process_landmark);

  sendTransaction(transaction);
}

beam::opt<Eigen::Vector3d>
    VisualOdometry::TriangulateLandmark(const uint64_t id) {
  std::vector<Eigen::Matrix4d, beam::AlignMat4d> T_cam_world_v;
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  beam_containers::Track track = landmark_container_->GetTrack(id);
  for (auto& m : track) {
    const auto T_world_camera = visual_map_->GetCameraPose(m.time_point);
    // check if the pose is in the graph
    if (T_world_camera.has_value()) {
      pixels.push_back(m.value.cast<int>());
      T_cam_world_v.push_back(T_world_camera.value().inverse());
    }
  }
  // triangulate new points
  if (T_cam_world_v.size() >= 5) {
    return beam_cv::Triangulation::TriangulatePoint(cam_model_, T_cam_world_v,
                                                    pixels);
  }
  return {};
}

void VisualOdometry::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  // ! known issue: if slam initialization doesn't recieve any visual
  // ! measurements, we need to set the first keyframe ourselves rather than
  // ! from the graph - use reference to most recent pose in graph to localize
  ROS_INFO_STREAM_ONCE("VisualOdometry: Received initial graph.");

  // all timestamps in the new graph
  const auto timestamps = bs_common::CurrentTimestamps(graph);

  // publish marginalized keyframes
  if (is_initialized_) {
    while (!keyframes_.empty() &&
           timestamps.find(keyframes_.front().Stamp()) == timestamps.end()) {
      PublishSlamChunk(keyframes_.front());
      keyframes_.pop_front();
    }
  }

  // Update graph object in visual map
  visual_map_->UpdateGraph(graph);

  // do initial setup
  if (!is_initialized_) {
    is_initialized_ = true;
    while (!visual_measurement_buffer_.empty()) {
      auto msg = visual_measurement_buffer_.front();
      if (msg->header.stamp < *timestamps.begin()) {
        visual_measurement_buffer_.pop_front();
        continue;
      }
      if (timestamps.find(msg->header.stamp) != timestamps.end()) {
        // measurement exists in graph -> therefore its a keyframe
        Keyframe kf(*msg);
        keyframes_.push_back(kf);
        previous_frame_ = msg->header.stamp;
        // remove measurement
        visual_measurement_buffer_.pop_front();
        continue;
      }
      // process message that isnt in graph yet
      const auto success = ComputeOdometryAndExtendMap(msg);
      if (!success) {
        // ! break and wait in normal loop for frame initializer to catch up
        break;
      }
      visual_measurement_buffer_.pop_front();
    }
  }
}

std::shared_ptr<fuse_constraints::AbsolutePose3DStampedConstraint>
    VisualOdometry::MakeFrameInitPrior(
        const ros::Time& frame_time,
        const Eigen::Matrix<double, 6, 6>& covariance) {
  const auto position = visual_map_->GetPosition(frame_time);
  const auto orientation = visual_map_->GetOrientation(frame_time);
  fuse_core::Vector7d mean;
  mean << position->x(), position->y(), position->z(), orientation->w(),
      orientation->x(), orientation->y(), orientation->z();
  return std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
      "FRAMEINITIALIZERPRIOR", *position, *orientation, mean, covariance);
}

void VisualOdometry::AddMeasurementsToContainer(
    const CameraMeasurementMsg::ConstPtr& msg) {
  // check that message hasnt already been added to container
  const auto times = landmark_container_->GetMeasurementTimes();
  if (times.find(msg->header.stamp.toNSec()) != times.end()) { return; }

  // put all measurements into landmark container
  for (const auto& lm : msg->landmarks) {
    Eigen::Vector2d landmark(static_cast<double>(lm.pixel_u),
                             static_cast<double>(lm.pixel_v));
    cv::Mat landmark_descriptor = beam_cv::Descriptor::VectorDescriptorToCvMat(
        {lm.descriptor.data}, msg->descriptor_type);
    beam_containers::LandmarkMeasurement lm_measurement(
        msg->header.stamp, msg->sensor_id, lm.landmark_id, msg->header.seq,
        landmark, landmark_descriptor);
    landmark_container_->Insert(lm_measurement);
  }
}

void VisualOdometry::GetPixelPointPairs(
    const ros::Time& img_time,
    std::vector<Eigen::Vector2i, beam::AlignVec2i>& pixels,
    std::vector<Eigen::Vector3d, beam::AlignVec3d>& points) {
  std::vector<std::pair<Eigen::Vector2i, Eigen::Vector3d>> pixel_point_pairs;
  std::vector<uint64_t> landmarks =
      landmark_container_->GetLandmarkIDsInImage(img_time);
  for (auto& id : landmarks) {
    fuse_variables::Point3DLandmark::SharedPtr lm =
        visual_map_->GetLandmark(id);
    if (lm) {
      Eigen::Vector3d point = lm->point();
      Eigen::Vector2i pixel =
          landmark_container_->GetValue(img_time, id).cast<int>();
      points.push_back(point);
      pixels.push_back(pixel);
    }
  }
}

void VisualOdometry::ComputeRelativeOdometry(
    const ros::Time& timestamp, const Eigen::Matrix4d& T_WORLD_BASELINKcur) {
  static uint64_t rel_odom_seq = 0;

  const auto prev_frame_pose = visual_map_->GetBaselinkPose(previous_frame_);
  const Eigen::Matrix4d T_WORLD_BASELINKprev = prev_frame_pose.value();

  const Eigen::Matrix4d T_PREVKF_CURFRAME =
      beam::InvertTransform(T_WORLD_BASELINKprev) * T_WORLD_BASELINKcur;
  const Eigen::Matrix4d T_ODOM_BASELINKcur =
      T_ODOM_BASELINKprev_ * T_PREVKF_CURFRAME;
  const auto odom_msg = bs_common::TransformToOdometryMessage(
      timestamp, rel_odom_seq++, extrinsics_.GetWorldFrameId(),
      extrinsics_.GetBaselinkFrameId(), T_ODOM_BASELINKcur);
  odometry_publisher_.publish(odom_msg);

  // update odom pose
  T_ODOM_BASELINKprev_ = T_ODOM_BASELINKcur;
}

void VisualOdometry::PublishSlamChunk(const Keyframe& keyframe) {
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
}

void VisualOdometry::PublishRelocRequest(const Keyframe& keyframe) {
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

} // namespace bs_models
