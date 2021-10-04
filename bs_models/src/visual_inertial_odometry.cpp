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
    : fuse_core::AsyncSensorModel(1), device_id_(fuse_core::uuid::NIL),
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
  for (auto &it : beam_cv::DescriptorTypeIntMap) {
    if (it.second == descriptor_type_) {
      descriptor_type_int_ = it.first;
    }
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
  init_odom_publisher_ = private_node_handle_.advertise<nav_msgs::Odometry>(
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
    const sensor_msgs::Image::ConstPtr &msg) {
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
      nav_msgs::Odometry odom;
      odom.pose.pose = pose.pose;
      odom.header.stamp = pose.header.stamp;
      odom.header.frame_id = extrinsics_.GetBaselinkFrameId();
      odom.child_frame_id = extrinsics_.GetWorldFrameId();
      init_odom_publisher_.publish(odom);

      // process keyframe
      if (IsKeyframe(img_time, T_WORLD_CAMERA)) {
        // update keyframe info
        Keyframe kf(img_time, image_buffer_.front());
        keyframes_.push_back(kf);
        added_since_kf_ = 0;

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

void VisualInertialOdometry::processIMU(const sensor_msgs::Imu::ConstPtr &msg) {
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
  // Update graph object in visual map
  visual_map_->UpdateGraph(graph);
}

void VisualInertialOdometry::SendInitializationGraph(
    const fuse_graphs::HashGraph &init_graph) {
  std::vector<uint64_t> new_landmarks;
  auto transaction = fuse_core::Transaction::make_shared();

  // add each variable in graph as they should be added
  for (auto &var : init_graph.getVariables()) {
    fuse_variables::Point3DLandmark::SharedPtr landmark =
        fuse_variables::Point3DLandmark::make_shared();
    fuse_variables::Position3DStamped::SharedPtr position =
        fuse_variables::Position3DStamped::make_shared();
    fuse_variables::Orientation3DStamped::SharedPtr orientation =
        fuse_variables::Orientation3DStamped::make_shared();

    if (var.type() == landmark->type()) {
      *landmark = dynamic_cast<const fuse_variables::Point3DLandmark &>(var);
      visual_map_->AddLandmark(landmark, transaction);
      new_landmarks.push_back(landmark->id());
    } else if (var.type() == orientation->type()) {
      *orientation =
          dynamic_cast<const fuse_variables::Orientation3DStamped &>(var);
      visual_map_->AddOrientation(orientation, transaction);
    } else if (var.type() == position->type()) {
      *position = dynamic_cast<const fuse_variables::Position3DStamped &>(var);
      visual_map_->AddPosition(position, transaction);
    } else {
      transaction->addVariable(std::move(var.clone()));
    }
  }

  // add each constraint in the graph
  for (auto &constraint : init_graph.getConstraints()) {
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
VisualInertialOdometry::LocalizeFrame(const ros::Time &img_time) {
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  std::vector<uint64_t> landmarks = tracker_->GetLandmarkIDsInImage(img_time);

  // get 2d-3d correspondences
  for (auto &id : landmarks) {
    fuse_variables::Point3DLandmark::SharedPtr lm =
        visual_map_->GetLandmark(id);
    if (lm) {
      Eigen::Vector3d point(lm->x(), lm->y(), lm->z());
      if (point.norm() < 1000) {
        Eigen::Vector2i pixeli = tracker_->Get(img_time, id).cast<int>();
        pixels.push_back(pixeli);
        points.push_back(point);
      } else {
        // remove landmark from graph
      }
    }
  }

  // get pose estimate from imu
  Eigen::Matrix4d T_WORLD_BASELINK_inertial;
  imu_preint_->GetPose(T_WORLD_BASELINK_inertial, img_time);

  // get previous two keyframe positions
  ros::Time prev_kf_time = (keyframes_[keyframes_.size() - 2]).Stamp();
  ros::Time cur_kf_time = keyframes_.back().Stamp();
  Eigen::Matrix4d T_prevkf = visual_map_->GetBaselinkPose(prev_kf_time).value();
  Eigen::Vector3d p_prevkf = T_prevkf.block<3, 1>(0, 3).transpose();
  Eigen::Matrix4d T_curkf = visual_map_->GetBaselinkPose(cur_kf_time).value();
  Eigen::Vector3d p_curkf = T_curkf.block<3, 1>(0, 3).transpose();

  // compute velocity from last two keyframe positions
  Eigen::Vector3d velocity =
      (p_curkf - p_prevkf) / (cur_kf_time.toSec() - prev_kf_time.toSec());

  // compute current position estimate assuming velocity is constant
  Eigen::Vector3d new_position =
      p_curkf + (img_time.toSec() - cur_kf_time.toSec()) * velocity;

  // combine imu rotation estimate and constant velocity position estimate
  Eigen::Matrix4d T_WORLD_BASELINK_estimate = Eigen::Matrix4d::Identity();
  T_WORLD_BASELINK_estimate.block<3, 3>(0, 0) =
      T_WORLD_BASELINK_inertial.block<3, 3>(0, 0);
  T_WORLD_BASELINK_estimate.block<3, 1>(0, 3) = new_position.transpose();

  // refine pose using motion only BA if there are enough points
  if (points.size() >= 15) {
    Eigen::Matrix4d T_CAMERA_WORLD_est =
        (T_WORLD_BASELINK_estimate * T_cam_baselink_.inverse()).inverse();
    Eigen::Matrix4d T_WORLD_CAMERA =
        pose_refiner_
            ->RefinePose(T_CAMERA_WORLD_est, cam_model_, pixels, points)
            .inverse();
    Eigen::Matrix4d T_WORLD_BASELINK = T_WORLD_CAMERA * T_cam_baselink_;

    // dont use refined estimate if its far from initial estimate
    if (beam::PassedMotionThreshold(T_WORLD_BASELINK_estimate, T_WORLD_BASELINK,
                                    10.0, 0.5)) {
      return T_WORLD_BASELINK_estimate;
    } else {
      return T_WORLD_BASELINK;
    }
  } else {
    // get pose estimate using imu preintegration
    return T_WORLD_BASELINK_estimate;
  }
}

bool VisualInertialOdometry::IsKeyframe(const ros::Time &img_time,
                                        const Eigen::Matrix4d &T_WORLD_CAMERA,
                                        bool compute_median) {
  // get pose of most recent keyframe
  ros::Time prev_kf_time = keyframes_.back().Stamp();
  Eigen::Matrix4d T_WORLD_CAM0 =
      visual_map_->GetCameraPose(prev_kf_time).value();

  // get relative rotation from current image to prev kf
  Eigen::Matrix4d T_CAM0_CAM1 = T_WORLD_CAM0.inverse() * T_WORLD_CAMERA;
  Eigen::Matrix3d R_CAM0_CAM1 = T_CAM0_CAM1.block(0, 0, 3, 3);

  // compute total parallax
  std::vector<uint64_t> frame1_ids = tracker_->GetLandmarkIDsInImage(img_time);
  std::vector<uint64_t> frame0_ids =
      tracker_->GetLandmarkIDsInImage(prev_kf_time);
  double total_parallax = 0.0;
  double num_correspondences = 0.0;
  std::vector<double> parallaxes;
  for (auto &id : frame1_ids) {
    try {
      // pixel in cam1 rotated into cam0
      Eigen::Vector2i p1 = tracker_->Get(img_time, id).cast<int>();
      Eigen::Vector3d bp1;
      cam_model_->BackProject(p1, bp1);
      Eigen::Vector3d adjusted_bp1 = R_CAM0_CAM1 * bp1;
      Eigen::Vector2d adjusted_p1;
      cam_model_->ProjectPoint(adjusted_bp1, adjusted_p1);

      // pixel in cam0
      Eigen::Vector2d p0 = tracker_->Get(prev_kf_time, id);
      double d = beam::distance(adjusted_p1, p0);
      if (compute_median) {
        parallaxes.push_back(d);
      } else {
        total_parallax += d;
      }
      num_correspondences += 1.0;
    } catch (const std::out_of_range &oor) {
    }
  }

  // compute average parallax (median or mean)
  double avg_parallax;
  if (compute_median) {
    std::sort(parallaxes.begin(), parallaxes.end());
    avg_parallax = parallaxes[parallaxes.size() / 2];
  } else {
    avg_parallax = total_parallax / num_correspondences;
  }

  if (avg_parallax > 5.0) {
    return true;
  }
  return false;
}

void VisualInertialOdometry::ExtendMap() {
  // get current and previous keyframe timestamps
  ros::Time prev_kf_time = (keyframes_[keyframes_.size() - 2]).Stamp();
  ros::Time cur_kf_time = keyframes_.back().Stamp();

  // make transaction
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(cur_kf_time);

  // add visual constraints
  std::vector<uint64_t> landmarks =
      tracker_->GetLandmarkIDsInImage(cur_kf_time);
  for (auto &id : landmarks) {
    fuse_variables::Point3DLandmark::SharedPtr lm =
        visual_map_->GetLandmark(id);

    // add constraints to triangulated ids
    if (lm) {
      visual_map_->AddConstraint(cur_kf_time, id,
                                 tracker_->Get(cur_kf_time, id), transaction);
    } else {

      // otherwise then triangulate then add the constraints
      std::vector<Eigen::Matrix4d, beam::AlignMat4d> T_cam_world_v;
      std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
      std::vector<ros::Time> observation_stamps;
      beam_cv::FeatureTrack track = tracker_->GetTrack(id);

      // triangulate features with long tracks
      if (track.size() > 5) {
        for (auto &m : track) {
          beam::opt<Eigen::Matrix4d> T =
              visual_map_->GetCameraPose(m.time_point);

          // check if the pose is in the graph (keyframe)
          if (T.has_value()) {
            pixels.push_back(m.value.cast<int>());
            T_cam_world_v.push_back(T.value().inverse());
            observation_stamps.push_back(m.time_point);
          }
        }

        if (T_cam_world_v.size() >= 2) {
          beam::opt<Eigen::Vector3d> point =
              beam_cv::Triangulation::TriangulatePoint(cam_model_,
                                                       T_cam_world_v, pixels);
          if (point.has_value()) {
            keyframes_.back().AddLandmark(id);
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

  ROS_INFO("Added %zu new landmarks.", keyframes_.back().Landmarks().size());

  // add inertial constraint
  AddInertialConstraint(transaction);

  // send transaction to graph
  sendTransaction(transaction);

  // publish new landmarks
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

  // get inertial constraint transaction
  fuse_core::Transaction::SharedPtr inertial_transaction =
      imu_preint_->RegisterNewImuPreintegratedFactor(
          cur_kf_time, img_orientation, img_position);

  // merge with existing transaction
  transaction->merge(*inertial_transaction);
}

void VisualInertialOdometry::NotifyNewKeyframe(
    const Eigen::Matrix4d &T_WORLD_CAMERA) {
  // send camera pose to graph
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(keyframes_.back().Stamp());
  visual_map_->AddCameraPose(T_WORLD_CAMERA, keyframes_.back().Stamp(),
                             transaction);
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
  std::vector<double> pose;
  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      pose.push_back(T_WORLD_BASELINK(i, j));
    }
  }
  reloc_msg.T_WORLD_BASELINK = pose;
  reloc_msg.stamp = keyframes_.back().Stamp();
  reloc_publisher_.publish(reloc_msg);
}

void VisualInertialOdometry::PublishSlamChunk() {
  // this just makes sure the visual map has the most recent variables
  for (auto &kf : keyframes_) {
    visual_map_->GetCameraPose(kf.Stamp());
  }

  // only once keyframes reaches the max window size, publish the keyframe
  if (keyframes_.size() == camera_params_.keyframe_window_size - 1) {
    // remove keyframe placeholder if first keyframe
    if (keyframes_.front().Stamp() == ros::Time(0)) {
      keyframes_.pop_front();
    }

    // build slam chunk
    bs_common::SlamChunkMsg slam_chunk;

    // stamp
    ros::Time kf_to_publish = keyframes_.front().Stamp();
    slam_chunk.stamp = kf_to_publish;

    // keyframe pose
    Eigen::Matrix4d T_WORLD_BASELINK =
        visual_map_->GetCameraPose(kf_to_publish).value() * T_cam_baselink_;

    // flatten 4x4 pose
    std::vector<double> pose;
    for (uint8_t i = 0; i < 3; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        pose.push_back(T_WORLD_BASELINK(i, j));
      }
    }
    slam_chunk.T_WORLD_BASELINK = pose;

    // trajectory
    TrajectoryMeasurementMsg trajectory;
    for (auto &it : keyframes_.front().Trajectory()) {
      // flatten 4x4 pose
      std::vector<double> pose;
      for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 4; j++) {
          pose.push_back(it.second(i, j));
        }
      }
      ros::Time stamp;
      stamp.fromNSec(it.first);
      trajectory.stamps.push_back(stamp.toSec());
      for (auto &x : pose) {
        trajectory.poses.push_back(x);
      }
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
    for (auto &id : landmark_ids) {
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
    const std::vector<uint64_t> &ids) {
  std_msgs::UInt64MultiArray landmark_msg;
  for (auto &id : ids) {
    landmark_msg.data.push_back(id);
  }
  landmark_publisher_.publish(landmark_msg);
}

} // namespace bs_models
