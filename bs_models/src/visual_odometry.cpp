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
  cam_model_->InitUndistortMap();
  visual_map_ = std::make_shared<VisualMap>(cam_model_);

  // Initialize landmark measurement container
  landmark_container_ = std::make_shared<beam_containers::LandmarkContainer>();

  // create pose refiner for motion only BA
  pose_refiner_ = std::make_shared<beam_cv::PoseRefinement>(1e-2, true, 1.0);

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

  // todo: attempt to process as many in buffer as possible so it doesnt fill up

  // retrieve and process the message at the front of the buffer
  const auto current_msg = visual_measurement_buffer_.front();
  const auto success = ComputeOdometryAndExtendMap(current_msg);

  // ! This should only fail if the frame initializer fails, in which
  // ! case we just need to wait until it succeeds
  if (!success) { return; }

  visual_measurement_buffer_.pop_front();

  // remove measurements from container if we are over the limit
  while (landmark_container_->NumImages() > vo_params_.max_container_size) {
    landmark_container_->PopFront();
  }
}

bool VisualOdometry::ComputeOdometryAndExtendMap(
    const CameraMeasurementMsg::ConstPtr& msg) {
  // odometry sequence numbers
  static uint64_t rel_odom_seq = 0;
  static uint64_t kf_odom_seq = 0;

  // estimate pose of frame wrt current graph
  Eigen::Matrix4d T_WORLD_BASELINKcur;
  const auto localization_success =
      LocalizeFrame(msg->header.stamp, T_WORLD_BASELINKcur);

  if (!localization_success) { return false; }

  // publish relative odometry
  const Eigen::Matrix4d T_WORLD_BASELINKprev =
      visual_map_->GetBaselinkPose(keyframes_.back().Stamp()).value();
  const Eigen::Matrix4d T_PREVKF_CURFRAME =
      beam::InvertTransform(T_WORLD_BASELINKprev) * T_WORLD_BASELINKcur;
  const Eigen::Matrix4d T_ODOM_BASELINKcur =
      T_ODOM_BASELINKprevkf_ * T_PREVKF_CURFRAME;
  const auto odom_msg = bs_common::TransformToOdometryMessage(
      msg->header.stamp, rel_odom_seq++, extrinsics_.GetWorldFrameId(),
      extrinsics_.GetBaselinkFrameId(), T_ODOM_BASELINKcur);
  odometry_publisher_.publish(odom_msg);

  // todo: add every pose to the graph even if not keyframe?
  // todo: otherwise IO will add poses itself?

  if (IsKeyframe(msg->header.stamp, T_WORLD_BASELINKcur)) {
    ROS_DEBUG_STREAM(
        "VisualOdometry: New keyframe detected at: " << msg->header.stamp);
    Keyframe kf(*msg);
    keyframes_.push_back(kf);
    ExtendMap(T_WORLD_BASELINKcur);

    // publish keyframe as odometry
    const auto kf_odom_msg = bs_common::TransformToOdometryMessage(
        kf.Stamp(), kf_odom_seq++, extrinsics_.GetWorldFrameId(),
        extrinsics_.GetBaselinkFrameId(), T_WORLD_BASELINKcur);
    keyframe_publisher_.publish(kf_odom_msg);

    // publish reloc request at given rate
    if ((kf.Stamp() - previous_reloc_request_).toSec() >
        vo_params_.reloc_request_period) {
      ROS_INFO_STREAM("Publishing reloc request at: " << kf.Stamp());
      previous_reloc_request_ = kf.Stamp();
      PublishRelocRequest(kf);
    }

    // update odom pose if its a keyframe
    T_ODOM_BASELINKprevkf_ = T_ODOM_BASELINKcur;
  }

  return true;
}

bool VisualOdometry::LocalizeFrame(const ros::Time& img_time,
                                   Eigen::Matrix4d& T_WORLD_BASELINK) {
  const auto prev_kf = keyframes_.back();
  Eigen::Matrix4d T_PREVKF_CURFRAME;
  if (!frame_initializer_->GetRelativePose(T_PREVKF_CURFRAME, prev_kf.Stamp(),
                                           img_time)) {
    ROS_WARN_STREAM("Unable to estimate pose from frame initializer, "
                    "buffering frame: "
                    << img_time);
    return false;
  }
  const auto prev_kf_pose = visual_map_->GetBaselinkPose(prev_kf.Stamp());
  if (!prev_kf_pose.has_value()) {
    ROS_ERROR_STREAM(__func__ << ": Cannot retrieve previous keyframe pose:"
                              << prev_kf.Stamp());
    throw std::runtime_error("Cannot retrieve previous keyframe pose.");
  }
  // estimate T_WORLD_BASELINK using the relative motion from frame init
  const Eigen::Matrix4d T_WORLD_BASELINKprev = prev_kf_pose.value();
  T_WORLD_BASELINK = T_WORLD_BASELINKprev * T_PREVKF_CURFRAME;

  // get 2d-3d correspondences
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  GetPixelPointPairs(img_time, pixels, points);
  // perform motion only BA to refine estimate
  if (pixels.size() >= 10) {
    Eigen::Matrix4d T_CAMERA_WORLD_est = beam::InvertTransform(
        T_WORLD_BASELINK * beam::InvertTransform(T_cam_baselink_));
    Eigen::Matrix4d T_CAMERA_WORLD_ref = pose_refiner_->RefinePose(
        T_CAMERA_WORLD_est, cam_model_, pixels, points);
    T_WORLD_BASELINK =
        beam::InvertTransform(T_CAMERA_WORLD_ref) * T_cam_baselink_;
  }

  return true;
}

bool VisualOdometry::IsKeyframe(const ros::Time& img_time,
                                const Eigen::Matrix4d& T_WORLD_BASELINK) {
  if (keyframes_.empty()) { return true; }

  const auto kf_time = keyframes_.back().Stamp();
  const Eigen::Matrix4d kf_pose = visual_map_->GetBaselinkPose(kf_time).value();

  if (vo_params_.use_parallax) {
    // check for parallax
    const auto avg_parallax =
        landmark_container_->ComputeParallax(kf_time, img_time, false);
    if (avg_parallax > vo_params_.keyframe_parallax) { return true; }
  } else {
    // check for movement
    const auto passed_motion = beam::PassedMotionThreshold(
        T_WORLD_BASELINK, kf_pose, vo_params_.keyframe_rotation_deg,
        vo_params_.keyframe_translation_m, true);
    if (passed_motion) { return true; }
  }

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
  visual_map_->AddBaselinkPose(T_WORLD_BASELINK, cur_kf_time, transaction);

  // add prior if using a frame initializer
  if (frame_initializer_ && vo_params_.use_pose_priors) {
    auto prior = MakeFrameInitPrior(cur_kf_time, vo_params_.prior_covariance);
    transaction->addConstraint(prior);
  }

  // add visual constraints
  std::vector<uint64_t> landmarks =
      landmark_container_->GetLandmarkIDsInImage(cur_kf_time);
  int num_lms_added = 0;
  // TODO: make body a lamba, use for each
  for (auto& id : landmarks) {
    fuse_variables::Point3DLandmark::SharedPtr lm =
        visual_map_->GetLandmark(id);
    // add constraints to triangulated ids
    if (lm) {
      Eigen::Vector2d pixel = landmark_container_->GetValue(cur_kf_time, id);
      Eigen::Vector2i tmp;
      if (!cam_model_->UndistortPixel(pixel.cast<int>(), tmp)) continue;
      // add constraint if its valid
      visual_map_->AddVisualConstraint(cur_kf_time, id, pixel, transaction);
    } else {
      // otherwise then triangulate and add the constraints
      std::vector<Eigen::Matrix4d, beam::AlignMat4d> T_cam_world_v;
      std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
      std::vector<ros::Time> observation_stamps;
      // get measurements of landmark for triangulation
      // TODO: iterate backwards through keyframes and stop after N matches
      for (auto& kf : keyframes_) {
        try {
          Eigen::Vector2d pixel = landmark_container_->GetValue(kf.Stamp(), id);
          Eigen::Vector2i tmp;
          if (!cam_model_->UndistortPixel(pixel.cast<int>(), tmp)) continue;
          beam::opt<Eigen::Matrix4d> T = visual_map_->GetCameraPose(kf.Stamp());
          if (T.has_value()) {
            pixels.push_back(pixel.cast<int>());
            T_cam_world_v.push_back(T.value().inverse());
            observation_stamps.push_back(kf.Stamp());
          }
        } catch (const std::out_of_range& oor) {}
      }

      // triangulate new points
      if (T_cam_world_v.size() >= 3) {
        beam::opt<Eigen::Vector3d> point =
            beam_cv::Triangulation::TriangulatePoint(cam_model_, T_cam_world_v,
                                                     pixels);
        if (point.has_value()) {
          visual_map_->AddLandmark(point.value(), id, transaction);
          num_lms_added++;
          for (int i = 0; i < observation_stamps.size(); i++) {
            visual_map_->AddVisualConstraint(
                observation_stamps[i], id,
                landmark_container_->GetValue(observation_stamps[i], id),
                transaction);
          }
        }
      }
    }
  }
  ROS_DEBUG_STREAM("Added " << num_lms_added << " new landmarks.");
  sendTransaction(transaction);
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

void VisualOdometry::PublishSlamChunk(const Keyframe& keyframe) {
  static uint64_t slam_chunk_seq = 0;
  const Eigen::Matrix4d T_WORLD_BASELINK =
      visual_map_->GetBaselinkPose(keyframe.Stamp()).value();
  SlamChunkMsg slam_chunk_msg;
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
