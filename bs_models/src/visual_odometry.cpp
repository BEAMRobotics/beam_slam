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

  // Load camera model and create visua map object
  cam_model_ = beam_calibration::CameraModel::Create(
      calibration_params_.cam_intrinsics_path);
  cam_model_->InitUndistortMap();
  visual_map_ = std::make_shared<VisualMap>(cam_model_);

  // Initialize landmark measurement container
  landmark_container_ = std::make_shared<beam_containers::LandmarkContainer>();

  // create pose refiner for motion only BA
  pose_refiner_ = std::make_shared<beam_cv::PoseRefinement>(1e-2, true, 1.0);

  // placeholder keyframe
  sensor_msgs::Image image;
  Keyframe kf(ros::Time(0), image);
  keyframes_.push_back(kf);
}

void VisualOdometry::onStart() {
  measurement_subscriber_ =
      private_node_handle_.subscribe<CameraMeasurementMsg>(
          ros::names::resolve(vo_params_.visual_measurement_topic), 100,
          &ThrottledMeasurementCallback::callback,
          &throttled_measurement_callback_,
          ros::TransportHints().tcpNoDelay(false));
}

void VisualOdometry::processMeasurements(
    const CameraMeasurementMsg::ConstPtr& msg) {
  AddMeasurementsToContainer(msg);
  // get most recent extrinsics, if failure then process frame later
  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get camera to baselink transform.");
    return;
  }

  // don't process until we have initialized
  if (!is_initialized_) { return; }

  // ! buffer the frame, attempt to process the front of the buffer

  // estimate pose of frame
  Eigen::Matrix4d T_WORLD_BASELINK = LocalizeFrame(msg->header.stamp);
  if (IsKeyframe(msg->header.stamp, T_WORLD_BASELINK)) {
    Keyframe kf(msg->header.stamp, msg->image);
    keyframes_.push_back(kf);
    ExtendMap(T_WORLD_BASELINK);
    if (keyframes_.size() >= 20) { keyframes_.pop_front(); }
  }

  // TODO: publish frame odometry
  // TODO: publish keyframe odometry
  // TODO: pop keyframes when we publish them as chunks

  // remove first image from container if we are over the limit
  if (landmark_container_->NumImages() > vo_params_.max_container_size) {
    landmark_container_->PopFront();
  }
}

void VisualOdometry::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  if (!is_initialized_) { is_initialized_ = true; }
  // Update graph object in visual map
  visual_map_->UpdateGraph(graph);
  // TODO: "process" all camera measurements that arent in the init graph, but
  // have been stored in the container
}

Eigen::Matrix4d VisualOdometry::LocalizeFrame(const ros::Time& img_time) {
  // get 2d-3d correspondences
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  GetPixelPointPairs(img_time, pixels, points);

  // get initial pose estimate
  Eigen::Matrix4d T_WORLD_BASELINK;
  if (frame_initializer_) { // get pose if we are using frame initializer
    if (!frame_initializer_->GetPose(T_WORLD_BASELINK, img_time,
                                     extrinsics_.GetBaselinkFrameId())) {
      ROS_WARN_STREAM("Unable to estimate pose from frame initializer, "
                      "buffering frame: "
                      << img_time);
      // TODO: return flag to show that it failed
    }
  } else {
    T_WORLD_BASELINK = beam_cv::AbsolutePoseEstimator::RANSACEstimator(
                           cam_model_, pixels, points)
                           .inverse();
  }

  // refine estimate if we have enough points to do so
  if (pixels.size() >= 10) {
    Eigen::Matrix4d T_CAMERA_WORLD_est =
        (T_WORLD_BASELINK * T_cam_baselink_.inverse()).inverse();
    Eigen::Matrix4d T_WORLD_CAMERA =
        pose_refiner_
            ->RefinePose(T_CAMERA_WORLD_est, cam_model_, pixels, points)
            .inverse();
    T_WORLD_BASELINK = T_WORLD_CAMERA * T_cam_baselink_;
  }

  // return estimate
  return T_WORLD_BASELINK;
}

bool VisualOdometry::IsKeyframe(const ros::Time& img_time,
                                const Eigen::Matrix4d& T_WORLD_BASELINK) {
  if ((img_time - keyframes_.back().Stamp()).toSec() >= 0.1) { return true; }
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
    auto prior = MakeFrameInitPrior(*visual_map_->GetPosition(cur_kf_time),
                                    *visual_map_->GetOrientation(cur_kf_time),
                                    vo_params_.prior_covariance);
    transaction->addConstraint(prior);
  }

  // add visual constraints
  std::vector<uint64_t> landmarks =
      landmark_container_->GetLandmarkIDsInImage(cur_kf_time);
  // TODO: make body a lamba, the use for each
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
          keyframes_.back().AddLandmark(id);
          visual_map_->AddLandmark(point.value(), id, transaction);
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
  ROS_INFO_STREAM("Added " << keyframes_.back().Landmarks().size()
                           << " new landmarks.");
  sendTransaction(transaction);
}

std::shared_ptr<fuse_constraints::AbsolutePose3DStampedConstraint>
    VisualOdometry::MakeFrameInitPrior(
        const fuse_variables::Position3DStamped& position,
        const fuse_variables::Orientation3DStamped& orientation,
        const Eigen::Matrix<double, 6, 6>& covariance) {
  fuse_core::Vector7d mean;
  mean << position.x(), position.y(), position.z(), orientation.w(),
      orientation.x(), orientation.y(), orientation.z();
  return std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
      "FRAMEINITIALIZERPRIOR", position, orientation, mean, covariance);
}

void VisualOdometry::AddMeasurementsToContainer(
    const CameraMeasurementMsg::ConstPtr& msg) {
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

} // namespace bs_models
