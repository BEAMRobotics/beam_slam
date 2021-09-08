#include <bs_models/visual_inertial_odometry.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/UInt64MultiArray.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_cv/matchers/Matchers.h>

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
  camera_params_.loadFromROS(private_node_handle_);
  calibration_params_.loadFromROS();

  /***********************************************************
   *       Initialize pose refiner object with params        *
   ***********************************************************/
  pose_refiner_ = std::make_shared<beam_cv::PoseRefinement>(1e-3);

  /***********************************************************
   *        Load camera model and Create Map object          *
   ***********************************************************/
  cam_model_ = beam_calibration::CameraModel::Create(
      calibration_params_.cam_intrinsics_path);
  visual_map_ = std::make_shared<VisualMap>(
      cam_model_, camera_params_.source, camera_params_.num_features_to_track,
      camera_params_.keyframe_window_size);

  /***********************************************************
   *              Initialize tracker variables               *
   ***********************************************************/
  descriptor_type_ =
      beam_cv::DescriptorTypeStringMap[camera_params_.descriptor];
  for (auto& it : beam_cv::DescriptorTypeIntMap) {
    if (it.second == descriptor_type_) { descriptor_type_int_ = it.first; }
  }
  std::shared_ptr<beam_cv::Descriptor> descriptor =
      beam_cv::Descriptor::Create(descriptor_type_);
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::GFTTDetector>(
          camera_params_.num_features_to_track);
  tracker_ = std::make_shared<beam_cv::KLTracker>(detector, descriptor,
                                                  camera_params_.window_size);

  /***********************************************************
   *               Create initializer object                 *
   ***********************************************************/
  initialization_ = std::make_shared<vision::VIOInitialization>(
      cam_model_, tracker_, camera_params_.init_path_topic,
      calibration_params_.imu_intrinsics_path, false,
      camera_params_.init_max_optimization_time_in_seconds,
      camera_params_.init_map_output_directory);

  // placeholder keyframe
  sensor_msgs::Image image;
  Keyframe kf(ros::Time(0), image);
  keyframes_.push_back(kf);
}

void VisualInertialOdometry::onStart() {
  /***********************************************************
   *                  Subscribe to topics                    *
   ***********************************************************/
  image_subscriber_ = node_handle_.subscribe<sensor_msgs::Image>(
      ros::names::resolve(camera_params_.image_topic), 1000,
      &ThrottledImageCallback::callback, &throttled_image_callback_,
      ros::TransportHints().tcpNoDelay(false));

  imu_subscriber_ = node_handle_.subscribe<sensor_msgs::Imu>(
      ros::names::resolve(camera_params_.imu_topic), 10000,
      &ThrottledIMUCallback::callback, &throttled_imu_callback_,
      ros::TransportHints().tcpNoDelay(false));

  /***********************************************************
   *                 Advertise publishers                    *
   ***********************************************************/
  init_odom_publisher_ =
      private_node_handle_.advertise<geometry_msgs::PoseStamped>(
          camera_params_.frame_odometry_output_topic, 10);
  new_keyframe_publisher_ = private_node_handle_.advertise<std_msgs::Header>(
      camera_params_.new_keyframes_topic, 10);
  slam_chunk_publisher_ = private_node_handle_.advertise<SlamChunkMsg>(
      camera_params_.slam_chunk_topic, 10);
  landmark_publisher_ =
      private_node_handle_.advertise<std_msgs::UInt64MultiArray>(
          camera_params_.landmark_topic, 10);
  reloc_publisher_ = private_node_handle_.advertise<RelocRequestMsg>(
      camera_params_.reloc_topic, 10);
}

void VisualInertialOdometry::processImage(
    const sensor_msgs::Image::ConstPtr& msg) {
  // get most recent extrinsics, if failure then process frame later
  if (!extrinsics_.GetT_CAMERA_BASELINK(T_cam_baselink_)) {
    ROS_ERROR("Unable to get camera to baselink transform.");
    return;
  }

  // push image onto buffer
  image_buffer_.push(*msg);

  // get current imu and image timestamps
  ros::Time imu_time = imu_buffer_.front().header.stamp;
  ros::Time img_time = image_buffer_.front().header.stamp;

  /**************************************************************************
   *                    Add Image to map or initializer                     *
   **************************************************************************/
  if (imu_time > img_time && !imu_buffer_.empty()) {
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
          ROS_INFO("Initialization Success: %f", img_time.toSec());

          // get the preintegration object
          imu_preint_ = initialization_->GetPreintegrator();

          // copy init graph and send to fuse optimizer
          SendInitializationGraph(initialization_->GetGraph());
        } else {
          ROS_INFO("Initialization Failure: %f", img_time.toSec());
        }
      }
    } else {
      // process in odometry mode
      beam::HighResolutionTimer frame_timer;

      // localize frame
      Eigen::Matrix4d T_WORLD_BASELINK = LocalizeFrame(img_time);
      Eigen::Matrix4d T_WORLD_CAMERA =
          T_WORLD_BASELINK * T_cam_baselink_.inverse();

      // publish pose to odom topic
      geometry_msgs::PoseStamped pose;
      bs_common::TransformationMatrixToPoseMsg(T_WORLD_BASELINK, img_time,
                                               pose);
      init_odom_publisher_.publish(pose);

      // process keyframe
      if (IsKeyframe(img_time, T_WORLD_BASELINK)) {
        // update keyframe info
        Keyframe kf(img_time, image_buffer_.front());
        keyframes_.push_back(kf);
        added_since_kf_ = 0;

        // log pose info
        ROS_INFO("Estimated Keyframe Pose:");
        std::cout << T_WORLD_BASELINK << std::endl;

        // notify that a new keyframe is detected
        NotifyNewKeyframe(T_WORLD_CAMERA);

        // extend map
        ExtendMap();

        // publish oldest keyframe for global mapper
        PublishSlamChunk();
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
      ROS_DEBUG("Total time to process frame: %.5f", frame_timer.elapsed());
    }
    image_buffer_.pop();
  }
}

void VisualInertialOdometry::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  // push imu message onto buffer
  imu_buffer_.push(*msg);

  // get current image timestamp
  ros::Time img_time = image_buffer_.front().header.stamp;

  /**************************************************************************
   *          Add IMU messages to preintegrator or initializer              *
   **************************************************************************/
  while (imu_buffer_.front().header.stamp <= img_time && !imu_buffer_.empty()) {
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
  visual_map_->UpdateGraph(graph);
}

void VisualInertialOdometry::SendInitializationGraph(
    const fuse_graphs::HashGraph& init_graph) {
  std::vector<uint64_t> new_landmarks;
  auto transaction = fuse_core::Transaction::make_shared();

  // add each variable in graph as they should be added
  for (auto& var : init_graph.getVariables()) {
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
  for (auto& constraint : init_graph.getConstraints()) {
    transaction->addConstraint(std::move(constraint.clone()));
  }

  // send transaction to graph
  sendTransaction(transaction);

  // announce the first keyframe
  std_msgs::Header keyframe_header;
  keyframe_header.stamp = keyframes_.back().Stamp();
  keyframe_header.frame_id = calibration_params_.baselink_frame;
  keyframe_header.seq = keyframes_.back().SequenceNumber();
  new_keyframe_publisher_.publish(keyframe_header);

  // publish landmarks
  PublishLandmarkIDs(new_landmarks);
}

Eigen::Matrix4d
    VisualInertialOdometry::LocalizeFrame(const ros::Time& img_time) {
  std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam_cv::AlignVec3d> points;
  std::vector<uint64_t> landmarks = tracker_->GetLandmarkIDsInImage(img_time);

  // get 2d-3d correspondences
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

  // refine pose using motion only BA if there are enough points
  if (points.size() >= 15) {
    Eigen::Matrix4d T_CAMERA_WORLD_est =
        beam_cv::AbsolutePoseEstimator::RANSACEstimator(cam_model_, pixels,
                                                        points, 500);
    Eigen::Matrix4d T_WORLD_CAMERA =
        pose_refiner_
            ->RefinePose(T_CAMERA_WORLD_est, cam_model_, pixels, points)
            .inverse();
    Eigen::Matrix4d T_WORLD_BASELINK = T_WORLD_CAMERA * T_cam_baselink_;
    return T_WORLD_BASELINK;
  } else {
    // get pose estimate using imu preintegration
    Eigen::Matrix4d T_WORLD_BASELINK_inertial;
    imu_preint_->GetPose(T_WORLD_BASELINK_inertial, img_time);
    return T_WORLD_BASELINK_inertial;
  }
}

bool VisualInertialOdometry::IsKeyframe(
    const ros::Time& img_time, const Eigen::Matrix4d& T_WORLD_BASELINK) {
  ros::Time prev_kf_time = keyframes_.back().Stamp();
  Eigen::Matrix4d T_prevkf = visual_map_->GetBaselinkPose(prev_kf_time).value();
  double parallax = tracker_->ComputeParallax(img_time, prev_kf_time, true);

  std::vector<uint64_t> landmarks = tracker_->GetLandmarkIDsInImage(img_time);
  std::vector<uint64_t> triangulated_landmarks;
  for (auto& id : landmarks) {
    if (visual_map_->GetLandmark(id)) { triangulated_landmarks.push_back(id); }
  }
  ROS_DEBUG("Parallax: %f, Triangulated/Total Tracks: %zu/%zu", parallax,
            triangulated_landmarks.size(), landmarks.size());

  if (beam::PassedMotionThreshold(T_WORLD_BASELINK, T_prevkf, 0.0, 0.05, true,
                                  true)) {
    return true;
  } else if (parallax > 50.0) {
    return true;
  } else if (triangulated_landmarks.size() < 30) {
    return true;
  } else if (img_time.toSec() - prev_kf_time.toSec() >= 3.0) {
    return true;
  }
  return false;
}

void VisualInertialOdometry::ExtendMap() {
  // get current and previous keyframe timestamps
  ros::Time prev_kf_time = (keyframes_[keyframes_.size() - 2]).Stamp();
  ros::Time cur_kf_time = keyframes_.back().Stamp();
  std::vector<uint64_t> new_landmarks;

  // make transaction
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(cur_kf_time);

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
      // otherwise then triangulate then add the constraints
      std::vector<Eigen::Matrix4d, beam_cv::AlignMat4d> T_cam_world_v;
      std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
      std::vector<ros::Time> observation_stamps;
      beam_cv::FeatureTrack track = tracker_->GetTrack(id);
      if (track.size() > 5) {
        for (auto& m : track) {
          beam::opt<Eigen::Matrix4d> T =
              visual_map_->GetCameraPose(m.time_point);

          // check if the pose is in the graph (keyframe)
          if (T.has_value()) {
            pixels.push_back(m.value.cast<int>());
            T_cam_world_v.push_back(T.value().inverse());
            observation_stamps.push_back(m.time_point);
          }
        }

        if (T_cam_world_v.size() >= 5) {
          beam::opt<Eigen::Vector3d> point =
              beam_cv::Triangulation::TriangulatePoint(cam_model_,
                                                       T_cam_world_v, pixels);
          if (point.has_value()) {
            new_landmarks.push_back(id);
            visual_map_->AddLandmark(point.value(), id, transaction);
            for (int i = 0; i < observation_stamps.size(); i++) {
              visual_map_->AddConstraint(
                  observation_stamps[i], id,
                  tracker_->Get(observation_stamps[i], id), transaction);
            }
          }
        }
      }
    }
  }

  ROS_INFO("Added %zu new landmarks.", new_landmarks.size());

  // add inertial constraint
  AddInertialConstraint(transaction);

  // send transaction to graph
  sendTransaction(transaction);

  // publish new landmarks
  PublishLandmarkIDs(new_landmarks);
}

void VisualInertialOdometry::AddInertialConstraint(
    fuse_core::Transaction::SharedPtr transaction) {
  ros::Time cur_kf_time = keyframes_.back().Stamp();

  // get robot pose variables at timestamp
  fuse_variables::Orientation3DStamped::SharedPtr img_orientation =
      visual_map_->GetOrientation(cur_kf_time);
  fuse_variables::Position3DStamped::SharedPtr img_position =
      visual_map_->GetPosition(cur_kf_time);

  // get inertial constraint transaction
  fuse_core::Transaction::SharedPtr inertial_transaction =
      imu_preint_->RegisterNewImuPreintegratedFactor(
          cur_kf_time, img_orientation, img_position);

  // merge with existing transaction
  transaction->merge(*inertial_transaction);
}

void VisualInertialOdometry::NotifyNewKeyframe(
    const Eigen::Matrix4d& T_WORLD_CAMERA) {
  // send keyframe pose to graph
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(keyframes_.back().Stamp());
  visual_map_->AddPose(T_WORLD_CAMERA, keyframes_.back().Stamp(), transaction);
  sendTransaction(transaction);

  // build header message and publish for lidar slam
  std_msgs::Header keyframe_header;
  keyframe_header.stamp = keyframes_.back().Stamp();
  keyframe_header.frame_id = calibration_params_.baselink_frame;
  keyframe_header.seq = keyframes_.back().SequenceNumber();
  new_keyframe_publisher_.publish(keyframe_header);

  // make and publish reloc request
  bs_common::RelocRequestMsg reloc_msg;
  reloc_msg.image = keyframes_.back().Image();
  Eigen::Matrix4d T_WORLD_BASELINK = T_WORLD_CAMERA * T_cam_baselink_;
  std::vector<float> pose;
  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      pose.push_back(static_cast<float>(T_WORLD_BASELINK(i, j)));
    }
  }
  reloc_msg.T_WORLD_BASELINK = pose;
  reloc_msg.stamp = keyframes_.back().Stamp();
  reloc_publisher_.publish(reloc_msg);
}

void VisualInertialOdometry::PublishSlamChunk() {
  // this just makes sure the visual map has the most recent variables
  for (auto& kf : keyframes_) { visual_map_->GetCameraPose(kf.Stamp()); }

  // only once keyframes reaches the max window size, publish the keyframe
  if (keyframes_.size() == camera_params_.keyframe_window_size - 1) {
    // remove keyframe placeholder if first keyframe
    if (keyframes_.front().Stamp() == ros::Time(0)) { keyframes_.pop_front(); }

    // build slam chunk
    bs_common::SlamChunkMsg slam_chunk;

    // stamp
    ros::Time kf_to_publish = keyframes_.front().Stamp();
    slam_chunk.stamp = kf_to_publish;

    // keyframe pose
    Eigen::Matrix4d T_WORLD_BASELINK =
        visual_map_->GetCameraPose(kf_to_publish).value() * T_cam_baselink_;

    // flatten 4x4 pose
    std::vector<float> pose;
    for (uint8_t i = 0; i < 3; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        pose.push_back(static_cast<float>(T_WORLD_BASELINK(i, j)));
      }
    }
    slam_chunk.T_WORLD_BASELINK = pose;

    // trajectory
    TrajectoryMeasurementMsg trajectory;
    for (auto& it : keyframes_.front().Trajectory()) {
      // flatten 4x4 pose
      std::vector<float> pose;
      for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 4; j++) {
          pose.push_back(static_cast<float>(it.second(i, j)));
        }
      }
      ros::Time stamp;
      stamp.fromNSec(it.first);
      trajectory.stamps.push_back(stamp.toSec());
      for (auto& x : pose) { trajectory.poses.push_back(x); }
    }
    slam_chunk.trajectory_measurement = trajectory;

    // camera measurements
    CameraMeasurementMsg camera_measurement;
    camera_measurement.descriptor_type = descriptor_type_int_;
    camera_measurement.sensor_id = 0;
    camera_measurement.measurement_id = keyframes_.front().SequenceNumber();
    camera_measurement.image = keyframes_.front().Image();

    // landmark measurements
    std::vector<LandmarkMeasurementMsg> landmarks;
    std::vector<uint64_t> landmark_ids =
        tracker_->GetLandmarkIDsInImage(kf_to_publish);
    for (auto& id : landmark_ids) {
      LandmarkMeasurementMsg lm;
      lm.landmark_id = id;
      cv::Mat descriptor = tracker_->GetDescriptor(kf_to_publish, id);
      std::vector<float> descriptor_v =
          beam_cv::Descriptor::ConvertDescriptor(descriptor, descriptor_type_);
      lm.descriptor = descriptor_v;
      Eigen::Vector2d pixel = tracker_->Get(kf_to_publish, id);
      lm.pixel_u = pixel[0];
      lm.pixel_v = pixel[1];
      landmarks.push_back(lm);
    }

    camera_measurement.landmarks = landmarks;
    slam_chunk.camera_measurement = camera_measurement;

    // publish slam chunk
    slam_chunk_publisher_.publish(slam_chunk);

    // remove keyframe
    keyframes_.pop_front();
  }
}

void VisualInertialOdometry::PublishLandmarkIDs(
    const std::vector<uint64_t>& ids) {
  std_msgs::UInt64MultiArray landmark_msg;
  for (auto& id : ids) { landmark_msg.data.push_back(id); }
  landmark_publisher_.publish(landmark_msg);
}

std::map<uint64_t, Eigen::Vector3d>
    VisualInertialOdometry::MatchFrameToCurrentSubmap(
        const ros::Time& img_time) {
  // vector of points to return
  std::map<uint64_t, Eigen::Vector3d> matched_points;

  // get map points in current camera frame
  Eigen::Matrix4d T_WORLD_CAMERA = visual_map_->GetCameraPose(img_time).value();
  std::vector<Eigen::Vector3d> points_camera =
      submap_.GetVisualMapPoints(T_WORLD_CAMERA);
  std::vector<cv::Mat> descriptors = submap_.GetDescriptors();

  // if submap empty then true empty vector
  if (points_camera.size() == 0) { return matched_points; }

  std::vector<cv::KeyPoint> projected_keypoints, current_keypoints;
  cv::Mat projected_descriptors, current_descriptors;

  // project each point to get keypoints
  for (size_t i = 0; i < points_camera.size(); i++) {
    Eigen::Vector3d point = points_camera[i];

    // project point into image plane
    bool in_image = false;
    Eigen::Vector2d pixel;
    cam_model_->ProjectPoint(point, pixel, in_image);

    // make cv keypoint for pixel
    if (in_image) {
      cv::KeyPoint kp;
      kp.pt.x = (float)pixel[0];
      kp.pt.y = (float)pixel[1];

      // get descriptor
      cv::Mat desc = descriptors[i];

      // push to results
      projected_descriptors.push_back(desc);
      projected_keypoints.push_back(kp);
    }
  }

  std::vector<uint64_t> untriangulated_ids;
  std::vector<uint64_t> landmarks = tracker_->GetLandmarkIDsInImage(img_time);
  for (auto& id : landmarks) {
    if (!visual_map_->GetLandmark(id) && !visual_map_->GetFixedLandmark(id)) {
      Eigen::Vector2d pixel = tracker_->Get(img_time, id);
      cv::KeyPoint kp;
      kp.pt.x = (float)pixel[0];
      kp.pt.y = (float)pixel[1];
      cv::Mat desc = tracker_->GetDescriptor(img_time, id);
      current_descriptors.push_back(desc);
      current_keypoints.push_back(kp);
      untriangulated_ids.push_back(id);
    }
  }

  // match features
  std::shared_ptr<beam_cv::Matcher> matcher =
      std::make_shared<beam_cv::BFMatcher>(cv::NORM_HAMMING);
  std::vector<cv::DMatch> matches =
      matcher->MatchDescriptors(projected_descriptors, current_descriptors,
                                projected_keypoints, current_keypoints);
  std::cout << matches.size() << std::endl;

  // filter matches by pixel distance
  for (auto& m : matches) {
    Eigen::Vector3d p_camera = points_camera[m.queryIdx];

    // transform point back into world frame
    Eigen::Vector4d ph_camera{p_camera[0], p_camera[1], p_camera[2], 1};
    Eigen::Vector3d p_world = (T_WORLD_CAMERA * ph_camera).hnormalized();

    // remove match from submap
    submap_.RemoveVisualMapPoint(m.queryIdx);
    matched_points[untriangulated_ids[m.queryIdx]] = p_world;
  }
  return matched_points;
}

} // namespace bs_models
