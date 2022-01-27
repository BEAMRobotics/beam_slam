#include <bs_models/visual_inertial_odometry.h>

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/UInt64MultiArray.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <bs_common/utils.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::VisualInertialOdometry,
                       fuse_core::SensorModel)

namespace bs_models {

using namespace vision;

VisualInertialOdometry::VisualInertialOdometry()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_image_callback_(std::bind(&VisualInertialOdometry::processImage,
                                          this, std::placeholders::_1)),
      throttled_imu_callback_(std::bind(&VisualInertialOdometry::processIMU,
                                        this, std::placeholders::_1)) {}

void VisualInertialOdometry::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  vio_params_.loadFromROS(private_node_handle_);
  calibration_params_.loadFromROS();

  // init frame initializer if desired
  if (vio_params_.frame_initializer_type == "ODOMETRY") {
    frame_initializer_ =
        std::make_unique<frame_initializers::OdometryFrameInitializer>(
            vio_params_.frame_initializer_info, 100, 30,
            vio_params_.frame_initializer_sensor_frame_id);
  } else if (vio_params_.frame_initializer_type == "POSEFILE") {
    frame_initializer_ =
        std::make_unique<frame_initializers::PoseFileFrameInitializer>(
            vio_params_.frame_initializer_info);
  }

  // initialize pose refiner object with params
  pose_refiner_ = std::make_shared<beam_cv::PoseRefinement>(1e-2, true, 1.0);

  // Load camera model and Create Map object
  cam_model_ = beam_calibration::CameraModel::Create(
      calibration_params_.cam_intrinsics_path);
  visual_map_ =
      std::make_shared<VisualMap>(cam_model_, vio_params_.num_features_to_track,
                                  vio_params_.keyframe_window_size);

  // Get descriptor type
  descriptor_type_ = beam_cv::DescriptorTypeStringMap[vio_params_.descriptor];
  for (auto& it : beam_cv::DescriptorTypeIntMap) {
    if (it.second == descriptor_type_) { descriptor_type_int_ = it.first; }
  }

  // Initialize descriptor
  std::shared_ptr<beam_cv::Descriptor> descriptor = beam_cv::Descriptor::Create(
      descriptor_type_, vio_params_.descriptor_config);

  // Initialize detector
  std::shared_ptr<beam_cv::Detector> detector = beam_cv::Detector::Create(
      beam_cv::DetectorTypeStringMap[vio_params_.detector],
      vio_params_.detector_config);

  // Initialize tracker
  beam_cv::KLTracker::Params tracker_params;
  tracker_params.LoadFromJson(vio_params_.tracker_config);
  tracker_ = std::make_shared<beam_cv::KLTracker>(
      tracker_params, detector, descriptor, vio_params_.tracker_window_size);

  // Create initializer object
  initialization_ = std::make_shared<vision::VIOInitialization>(
      cam_model_, tracker_, vio_params_.init_path_topic,
      calibration_params_.imu_intrinsics_path,
      vio_params_.init_use_scale_estimate,
      vio_params_.init_max_optimization_time_in_seconds,
      vio_params_.init_map_output_directory);

  // placeholder keyframe
  sensor_msgs::Image image;
  Keyframe kf(ros::Time(0), image);
  keyframes_.push_back(kf);
}

void VisualInertialOdometry::onStart() {
  // subscribe to topics
  image_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Image>(
      ros::names::resolve(vio_params_.image_topic), 1000,
      &ThrottledImageCallback::callback, &throttled_image_callback_,
      ros::TransportHints().tcpNoDelay(false));
  imu_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Imu>(
      ros::names::resolve(vio_params_.imu_topic), 10000,
      &ThrottledIMUCallback::callback, &throttled_imu_callback_,
      ros::TransportHints().tcpNoDelay(false));

  // Advertise publishers
  init_odom_publisher_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odometry", 10);
  new_keyframe_publisher_ =
      private_node_handle_.advertise<std_msgs::Header>("keyframes", 10);
  landmark_publisher_ =
      private_node_handle_.advertise<std_msgs::UInt64MultiArray>("landmarks",
                                                                 10);
  slam_chunk_publisher_ = private_node_handle_.advertise<SlamChunkMsg>(
      "/local_mapper/slam_results", 10);
  reloc_publisher_ = private_node_handle_.advertise<RelocRequestMsg>(
      "/local_mapper/reloc_request", 10);
}

/************************************************************
 *                          Callbacks                       *
 ************************************************************/
void VisualInertialOdometry::processImage(
    const sensor_msgs::Image::ConstPtr& msg) {
  // push image onto buffer
  image_buffer_.push(*msg);

  // get current imu and image timestamps
  ros::Time imu_time = imu_buffer_.front().header.stamp;
  ros::Time img_time = image_buffer_.front().header.stamp;

  // get most recent extrinsics, if failure then process frame later
  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get camera to baselink transform.");
    return;
  }

  // get pose if we are using frame initializer, if its a failure we wait
  // until we can by buffering the frame
  Eigen::Matrix4d T_WORLD_BASELINK = Eigen::Matrix4d::Identity();
  if (frame_initializer_) {
    if (!frame_initializer_->GetEstimatedPose(
            T_WORLD_BASELINK, img_time, extrinsics_.GetBaselinkFrameId())) {
      ROS_WARN_STREAM("Unable to estimate pose from frame initializer, "
                      "buffering frame: "
                      << img_time);
      return;
    }
  }

  // add image to map or initializer
  if (imu_time >= img_time && !imu_buffer_.empty()) {
    // add image to tracker
    tracker_->AddImage(
        beam_cv::OpenCVConversions::RosImgToMat(image_buffer_.front()),
        img_time);

    // process in initialization mode
    if (!initialization_->Initialized()) {
      if ((img_time - keyframes_.back().Stamp()).toSec() >= 0.5) {
        Keyframe kf(img_time, image_buffer_.front());
        keyframes_.push_back(kf);
        added_since_kf_ = 0;
        if (initialization_->AddImage(img_time)) {
          ROS_INFO_STREAM("Initialization Success: " << img_time);

          // get the preintegration object
          imu_preint_ = initialization_->GetPreintegrator();

          // copy init graph and send to fuse optimizer
          SendInitializationGraph(initialization_->GetGraph());
        } else {
          ROS_INFO_STREAM("Initialization Failure: " << img_time);
        }
      }
    } else {
      // process in odometry mode
      beam::HighResolutionTimer frame_timer;

      // localize frame if not using a frame initializer
      if (!frame_initializer_) { T_WORLD_BASELINK = LocalizeFrame(img_time); }

      // detect if odometry has failed
      if (FailureDetection(img_time, T_WORLD_BASELINK)) {
        ROS_FATAL_STREAM("VIO Failure, reintializing at " << img_time);
        ros::requestShutdown();
        // TODO: publish reset request to reinitialize
      }

      // publish pose to odom topic
      PublishInitialOdometry(img_time, T_WORLD_BASELINK);

      // process keyframe
      if (IsKeyframe(img_time, T_WORLD_BASELINK)) {
        // push new keyframe to list
        Keyframe kf(img_time, image_buffer_.front());
        keyframes_.push_back(kf);
        added_since_kf_ = 0;

        // log keyframe info
        ROS_INFO_STREAM("Keyframe time: " << img_time);
        ROS_INFO_STREAM(beam::TransformationMatrixToString(T_WORLD_BASELINK));

        // update map with keyframe
        ExtendMap(T_WORLD_BASELINK);

        // notify system of new keyframe
        NotifyNewKeyframe(T_WORLD_BASELINK);
      } else {
        // compute relative pose to most recent kf
        Eigen::Matrix4d T_WORLD_BASELINK_curkf =
            visual_map_->GetBaselinkPose(keyframes_.back().Stamp()).value();
        Eigen::Matrix4d T_curframe_curkeyframe =
            T_WORLD_BASELINK.inverse() * T_WORLD_BASELINK_curkf;
        // add to current keyframes trajectory
        keyframes_.front().AddPose(img_time, T_curframe_curkeyframe);
        added_since_kf_++;
      }
      ROS_INFO_STREAM("Total time to process frame: " << frame_timer.elapsed());
    }
    image_buffer_.pop();
  }
}

void VisualInertialOdometry::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  // push imu message onto buffer
  imu_buffer_.push(*msg);

  // get current image timestamp
  ros::Time img_time = image_buffer_.front().header.stamp;

  // add IMU messages to preintegrator or initializer
  while (imu_buffer_.front().header.stamp < img_time && !imu_buffer_.empty()) {
    if (!initialization_->Initialized()) {
      initialization_->AddIMU(imu_buffer_.front());
    } else {
      imu_preint_->AddToBuffer(imu_buffer_.front());
    }
    imu_buffer_.pop();
  }
}

void VisualInertialOdometry::onGraphUpdate(
    fuse_core::Graph::ConstSharedPtr graph) {
  // make a copy to make updating graph easier
  fuse_core::Graph::SharedPtr copy = graph->clone();

  // publish marginalized slam chunks
  while (!keyframes_.empty() &&
         !copy->variableExists(
             visual_map_->GetPositionUUID(keyframes_.front().Stamp()))) {
    PublishSlamChunk(keyframes_.front());
    keyframes_.pop_front();
  }

  // Update graph object in visual map
  visual_map_->UpdateGraph(copy);

  // Update imu preint info with new graph
  imu_preint_->UpdateGraph(copy);
}

/************************************************************
 *                           Helpers                        *
 ************************************************************/
Eigen::Matrix4d
    VisualInertialOdometry::LocalizeFrame(const ros::Time& img_time) {
  // output pose
  Eigen::Matrix4d T_WORLD_BASELINK;

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

  // get pose estimate using imu
  std::shared_ptr<Eigen::Matrix<double, 6, 6>> covariance =
      std::make_shared<Eigen::Matrix<double, 6, 6>>();
  imu_preint_->GetPose(T_WORLD_BASELINK, img_time, covariance);

  // refine with visual info if possible
  if (pixels.size() >= 15) {
    Eigen::Matrix4d T_CAMERA_WORLD_est =
        (T_WORLD_BASELINK * T_cam_baselink_.inverse()).inverse();
    Eigen::Matrix4d T_WORLD_CAMERA =
        pose_refiner_
            ->RefinePose(T_CAMERA_WORLD_est, cam_model_, pixels, points,
                         covariance)
            .inverse();
    T_WORLD_BASELINK = T_WORLD_CAMERA * T_cam_baselink_;
  }
  // return estimate
  return T_WORLD_BASELINK;
}

bool VisualInertialOdometry::IsKeyframe(
    const ros::Time& img_time, const Eigen::Matrix4d& T_WORLD_BASELINK) {
  // get timestamp of last keyframe
  ros::Time kf_time = keyframes_.back().Stamp();

  // get pose of last keyframe
  Eigen::Matrix4d kf_pose = visual_map_->GetBaselinkPose(kf_time).value();

  double avg_parallax = tracker_->ComputeParallax(kf_time, img_time, true);
  ROS_DEBUG_STREAM("Computed parallax to last keyframe: " << avg_parallax);
  if (avg_parallax > vio_params_.keyframe_parallax &&
      beam::PassedMotionThreshold(T_WORLD_BASELINK, kf_pose, 0.0, 0.05, true,
                                  true)) {
    return true;
  } else if (added_since_kf_ > vio_params_.tracker_window_size - 10) {
    return true;
  }
  // TODO: add check for total # of tracks, if it falls below threshold then
  // make keyframe
  return false;
}

bool VisualInertialOdometry::FailureDetection(
    const ros::Time& img_time, const Eigen::Matrix4d& T_WORLD_BASELINK) {
  // check current number of visible tracks
  ros::Time kf_time = keyframes_.back().Stamp();
  std::vector<uint64_t> ids = tracker_->GetLandmarkIDsInImage(img_time);
  int num_tracks = 0;
  for (auto& id : ids) {
    try {
      tracker_->Get(kf_time, id).cast<int>();
      tracker_->Get(img_time, id).cast<int>();
      num_tracks++;
    } catch (const std::out_of_range& oor) {}
  }
  if (num_tracks < 30) {
    ROS_FATAL_STREAM("Too few visual tracks: " << num_tracks);
    return true;
  }

  // check imu biases
  if (imu_preint_->GetImuState().AccelBiasVec().norm() > 2.5) {
    ROS_FATAL_STREAM("Too large of accel bias: "
                     << imu_preint_->GetImuState().AccelBiasVec().norm());
    return true;
  }
  if (imu_preint_->GetImuState().GyroBiasVec().norm() > 1.0) {
    ROS_FATAL_STREAM("Too large of gryo bias: "
                     << imu_preint_->GetImuState().GyroBiasVec().norm());
    return true;
  }

  // check change in pose from last keyframe
  Eigen::Matrix4d kf_pose = visual_map_->GetBaselinkPose(kf_time).value();
  if (beam::PassedMotionThreshold(T_WORLD_BASELINK, kf_pose, 50.0, 5.0, true,
                                  false)) {
    ROS_FATAL_STREAM("Too large of a pose change.");
    return true;
  }

  // check change in z
  if (std::abs(T_WORLD_BASELINK(2, 3) - kf_pose(2, 3)) > 1) {
    ROS_FATAL_STREAM("Too large of a translation in z.");
    return true;
  }
  return false;
}

void VisualInertialOdometry::ExtendMap(
    const Eigen::Matrix4d& T_WORLD_BASELINK) {
  // get current and previous keyframe timestamp
  ros::Time prev_kf_time = (keyframes_[keyframes_.size() - 2]).Stamp();
  ros::Time cur_kf_time = keyframes_.back().Stamp();

  // create transaction for this keyframe
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(cur_kf_time);

  // add pose to map
  visual_map_->AddBaselinkPose(T_WORLD_BASELINK, cur_kf_time, transaction);

  // add prior if using a frame initializer
  if (frame_initializer_ && vio_params_.use_pose_priors) {
    auto p = visual_map_->GetPosition(cur_kf_time);
    auto o = visual_map_->GetOrientation(cur_kf_time);
    fuse_core::Vector7d mean;
    mean << p->x(), p->y(), p->z(), o->w(), o->x(), o->y(), o->z();
    auto prior =
        std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
            "FRAMEINITIALIZERPRIOR", *p, *o, mean,
            vio_params_.prior_covariance);
    transaction->addConstraint(prior);
  }

  // add visual constraints
  std::vector<uint64_t> landmarks =
      tracker_->GetLandmarkIDsInImage(cur_kf_time);
  for (auto& id : landmarks) {
    fuse_variables::Point3DLandmark::SharedPtr lm =
        visual_map_->GetLandmark(id);
    // add constraints to triangulated ids
    if (lm) {
      visual_map_->AddConstraint(cur_kf_time, id,
                                 tracker_->Get(cur_kf_time, id), transaction);
    } else {
      // triangulate using last keyframe
      try {
        // get pixel correspondence
        Eigen::Vector2i p1 = tracker_->Get(prev_kf_time, id).cast<int>();
        Eigen::Vector2i p2 = tracker_->Get(cur_kf_time, id).cast<int>();
        // get camera poses
        Eigen::Matrix4d T_cam1_world =
            visual_map_->GetCameraPose(prev_kf_time).value();
        Eigen::Matrix4d T_cam2_world =
            visual_map_->GetCameraPose(cur_kf_time).value();
        // triangulate
        beam::opt<Eigen::Vector3d> point =
            beam_cv::Triangulation::TriangulatePoint(
                cam_model_, cam_model_, T_cam1_world, T_cam2_world, p1, p2);

        if (point.has_value()) {
          keyframes_.back().AddLandmark(id);
          visual_map_->AddLandmark(point.value(), id, transaction);
          visual_map_->AddConstraint(
              cur_kf_time, id, tracker_->Get(cur_kf_time, id), transaction);
        }
      } catch (const std::out_of_range& oor) {}
    }
  }
  ROS_INFO_STREAM("Added " << keyframes_.back().Landmarks().size()
                           << " new landmarks.");
  AddInertialConstraint(transaction);
  sendTransaction(transaction);
  PublishLandmarkIDs(keyframes_.back().Landmarks());
}

void VisualInertialOdometry::AddInertialConstraint(
    fuse_core::Transaction::SharedPtr transaction) {
  ros::Time cur_kf_time = keyframes_.back().Stamp();
  // get robot pose variables at timestamp
  fuse_variables::Orientation3DStamped::SharedPtr img_orientation =
      visual_map_->GetOrientation(cur_kf_time);
  fuse_variables::Position3DStamped::SharedPtr img_position =
      visual_map_->GetPosition(cur_kf_time);

  // get inertial constraint transaction & merge
  fuse_core::Transaction::SharedPtr inertial_transaction =
      imu_preint_->RegisterNewImuPreintegratedFactor(
          cur_kf_time, img_orientation, img_position);
  transaction->merge(*inertial_transaction);
}

void VisualInertialOdometry::SendInitializationGraph(
    const fuse_core::Graph::SharedPtr& init_graph) {
  std::vector<uint64_t> new_landmarks;
  auto transaction = fuse_core::Transaction::make_shared();
  // add each variable in graph as they should be added
  for (auto& var : init_graph->getVariables()) {
    fuse_variables::Point3DLandmark::SharedPtr landmark =
        fuse_variables::Point3DLandmark::make_shared();
    fuse_variables::Position3DStamped::SharedPtr position =
        fuse_variables::Position3DStamped::make_shared();
    fuse_variables::Orientation3DStamped::SharedPtr orientation =
        fuse_variables::Orientation3DStamped::make_shared();
    if (var.type() == landmark->type()) {
      *landmark = dynamic_cast<const fuse_variables::Point3DLandmark&>(var);
      visual_map_->AddLandmark(landmark, transaction);
      new_landmarks.push_back(landmark->id());
    } else if (var.type() == orientation->type()) {
      *orientation =
          dynamic_cast<const fuse_variables::Orientation3DStamped&>(var);
      visual_map_->AddOrientation(orientation, transaction);
    } else if (var.type() == position->type()) {
      *position = dynamic_cast<const fuse_variables::Position3DStamped&>(var);
      visual_map_->AddPosition(position, transaction);
    } else {
      transaction->addVariable(std::move(var.clone()));
    }
  }

  // add each constraint in the graph
  for (auto& constraint : init_graph->getConstraints()) {
    transaction->addConstraint(std::move(constraint.clone()));
  }
  sendTransaction(transaction);

  // announce the first keyframe
  std_msgs::Header keyframe_header;
  keyframe_header.stamp = keyframes_.back().Stamp();
  keyframe_header.frame_id = calibration_params_.baselink_frame;
  keyframe_header.seq = keyframes_.back().SequenceNumber();
  new_keyframe_publisher_.publish(keyframe_header);

  PublishLandmarkIDs(new_landmarks);
}

/************************************************************
 *                     Publishing stuff                     *
 ************************************************************/
void VisualInertialOdometry::NotifyNewKeyframe(
    const Eigen::Matrix4d& T_WORLD_BASELINK) {
  // build header message and publish for lidar slam
  std_msgs::Header keyframe_header;
  keyframe_header.stamp = keyframes_.back().Stamp();
  keyframe_header.frame_id = calibration_params_.baselink_frame;
  keyframe_header.seq = keyframes_.back().SequenceNumber();
  new_keyframe_publisher_.publish(keyframe_header);

  // make and publish reloc request
  // TODO: redo this to only be published at a specified rate
  // TODO: refactor reloc message to take a camera measurement
  bs_common::RelocRequestMsg reloc_msg;
  reloc_msg.image = keyframes_.back().Image();
  std::vector<double> pose;
  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 4; j++) { pose.push_back(T_WORLD_BASELINK(i, j)); }
  }
  reloc_msg.T_WORLD_BASELINK = pose;
  reloc_msg.stamp = keyframes_.back().Stamp();
  reloc_publisher_.publish(reloc_msg);
}

void VisualInertialOdometry::PublishInitialOdometry(
    const ros::Time& time, const Eigen::Matrix4d& T_WORLD_BASELINK) {
  // build pose message
  geometry_msgs::PoseStamped pose;
  bs_common::TransformationMatrixToPoseMsg(T_WORLD_BASELINK, time, pose);

  // build odom message
  nav_msgs::Odometry odom;
  odom.pose.pose = pose.pose;
  odom.header.stamp = pose.header.stamp;
  odom.header.frame_id = extrinsics_.GetWorldFrameId();
  odom.child_frame_id = extrinsics_.GetBaselinkFrameId();

  init_odom_publisher_.publish(odom);
}

void VisualInertialOdometry::PublishSlamChunk(Keyframe keyframe) {
  // dont publish placeholder
  if (keyframe.Stamp() == ros::Time(0)) { return; }

  ros::Time keyframe_time = keyframe.Stamp();
  bs_common::SlamChunkMsg slam_chunk;

  // get keyframe pose and flatten it
  beam::opt<Eigen::Matrix4d> T_WORLD_BASELINK =
      visual_map_->GetBaselinkPose(keyframe_time);
  if (T_WORLD_BASELINK.has_value()) {
    std::vector<double> pose;
    for (uint8_t i = 0; i < 3; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        pose.push_back(T_WORLD_BASELINK.value()(i, j));
      }
    }
    slam_chunk.T_WORLD_BASELINK = pose;
  } else {
    ROS_WARN_STREAM("Attempting to publish slam chunk ["
                    << keyframe_time << "] that no longer exists in graph.");
    return;
  }

  // build trajectory msg
  TrajectoryMeasurementMsg trajectory;
  for (auto& it : keyframe.Trajectory()) {
    // flatten 4x4 pose
    std::vector<double> pose;
    for (uint8_t i = 0; i < 3; i++) {
      for (uint8_t j = 0; j < 4; j++) { pose.push_back(it.second(i, j)); }
    }
    ros::Time stamp;
    stamp.fromNSec(it.first);
    trajectory.stamps.push_back(stamp.toSec());
    for (auto& x : pose) { trajectory.poses.push_back(x); }
  }

  // build landmark measurements msg
  std::vector<LandmarkMeasurementMsg> landmarks;
  std::vector<uint64_t> landmark_ids =
      tracker_->GetLandmarkIDsInImage(keyframe_time);
  for (auto& id : landmark_ids) {
    LandmarkMeasurementMsg lm;
    lm.landmark_id = id;
    cv::Mat descriptor = tracker_->GetDescriptor(keyframe_time, id);
    std::vector<float> descriptor_v =
        beam_cv::Descriptor::ConvertDescriptor(descriptor, descriptor_type_);
    lm.descriptor = descriptor_v;
    Eigen::Vector2d pixel = tracker_->Get(keyframe_time, id);
    lm.pixel_u = pixel[0];
    lm.pixel_v = pixel[1];
    landmarks.push_back(lm);
  }

  // build camera measurement msg
  CameraMeasurementMsg camera_measurement;
  camera_measurement.descriptor_type = descriptor_type_int_;
  camera_measurement.sensor_id = 0;
  camera_measurement.measurement_id = keyframe.SequenceNumber();
  camera_measurement.image = keyframe.Image();
  camera_measurement.landmarks = landmarks;

  // add msgs to slam chunk
  slam_chunk.stamp = keyframe_time;
  slam_chunk.trajectory_measurement = trajectory;
  slam_chunk.camera_measurement = camera_measurement;

  slam_chunk_publisher_.publish(slam_chunk);
}

void VisualInertialOdometry::PublishLandmarkIDs(
    const std::vector<uint64_t>& ids) {
  std_msgs::UInt64MultiArray landmark_msg;
  for (auto& id : ids) { landmark_msg.data.push_back(id); }
  landmark_publisher_.publish(landmark_msg);
}

} // namespace bs_models
