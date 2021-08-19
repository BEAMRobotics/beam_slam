#include <bs_models/camera_to_camera/visual_inertial_odom.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

#include <nlohmann/json.hpp>

// messages
#include <bs_models/RelocRequestMsg.h>
#include <bs_models/SlamChunkMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt64MultiArray.h>

// libbeam
#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/Utils.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/Triangulation.h>

#include <bs_common/utils.h>
#include <bs_models/camera_to_camera/utils.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::camera_to_camera::VisualInertialOdom,
                       fuse_core::SensorModel)

namespace bs_models { namespace camera_to_camera {

VisualInertialOdom::VisualInertialOdom()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_image_callback_(std::bind(&VisualInertialOdom::processImage,
                                          this, std::placeholders::_1)),
      throttled_imu_callback_(std::bind(&VisualInertialOdom::processIMU, this,
                                        std::placeholders::_1)) {}

void VisualInertialOdom::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  camera_params_.loadFromROS(private_node_handle_);
  global_params_.loadFromROS(private_node_handle_);

  /***********************************************************
   *  Load camera model, create visual map and pose refiner  *
   ***********************************************************/
  cam_model_ =
      beam_calibration::CameraModel::Create(global_params_.cam_intrinsics_path);
  visual_map_ = std::make_shared<VisualMap>(
      cam_model_, camera_params_.source, camera_params_.num_features_to_track,
      camera_params_.keyframe_window_size);
  pose_refiner_ = bs_models::camera_to_camera::PoseRefiner();

  /***********************************************************
   *              Initialize tracker variables               *
   ***********************************************************/
  beam_cv::DescriptorType descriptor_type =
      beam_cv::DescriptorTypeStringMap[camera_params_.descriptor];
  std::shared_ptr<beam_cv::Descriptor> descriptor =
      beam_cv::Descriptor::Create(descriptor_type);
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::GFTTDetector>(
          camera_params_.num_features_to_track);
  tracker_ = std::make_shared<beam_cv::KLTracker>(detector, descriptor,
                                                  camera_params_.window_size);

  /***********************************************************
   *               Create initializer object                 *
   ***********************************************************/
  nlohmann::json J;
  std::ifstream file(global_params_.imu_intrinsics_path);
  file >> J;
  initializer_ = std::make_shared<bs_models::camera_to_camera::VIOInitializer>(
      cam_model_, tracker_, camera_params_.init_path_topic, J["cov_gyro_noise"],
      J["cov_accel_noise"], J["cov_gyro_bias"], J["cov_accel_bias"], false,
      camera_params_.init_max_optimization_time_in_seconds,
      camera_params_.init_map_output_directory);

  // placeholder keyframe
  sensor_msgs::Image image;
  bs_models::camera_to_camera::Keyframe kf(ros::Time(0), image);
  keyframes_.push_back(kf);
}

void VisualInertialOdom::onStart() {
  /***********************************************************
   *                  Subscribe to topics                    *
   ***********************************************************/
  image_subscriber_ = node_handle_.subscribe(camera_params_.image_topic, 1000,
                                             &ThrottledImageCallback::callback,
                                             &throttled_image_callback_);
  imu_subscriber_ = node_handle_.subscribe(camera_params_.imu_topic, 10000,
                                           &ThrottledIMUCallback::callback,
                                           &throttled_imu_callback_);
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
      camera_params_.reloc_request_topic, 10);
}

void VisualInertialOdom::processImage(const sensor_msgs::Image::ConstPtr& msg) {
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
    if (!initializer_->Initialized()) {
      if ((img_time - keyframes_.back().Stamp()).toSec() >= 1.0) {
        bs_models::camera_to_camera::Keyframe kf(img_time,
                                                 image_buffer_.front());
        keyframes_.push_back(kf);
        added_since_kf_ = 0;
        if (initializer_->AddImage(img_time)) {
          ROS_INFO("Initialization Success: %f", img_time.toSec());
          // get the preintegration object
          imu_preint_ = initializer_->GetPreintegrator();
          // copy init graph and send to fuse optimizer
          SendInitializationGraph(initializer_->GetGraph());
        } else {
          ROS_INFO("Initialization Failure: %f", img_time.toSec());
        }
      }
    } else { // process in odometry mode
      beam::HighResolutionTimer frame_timer;

      // localize frame
      beam::opt<Eigen::Matrix4d> T_WORLD_CAMERA =
          bs_models::camera_to_camera::LocalizeFrame(
              tracker_, visual_map_, pose_refiner_, cam_model_, img_time);
      Eigen::Matrix4d T_WORLD_BASELINK;
      if (T_WORLD_CAMERA.has_value()) {
        T_WORLD_BASELINK = T_WORLD_CAMERA.value() * T_cam_baselink_;
      } else {
        // TODO: get pose from imu
      }

      // publish pose to odom topic
      geometry_msgs::PoseStamped pose;
      bs_common::TransformationMatrixToPoseMsg(T_WORLD_BASELINK, img_time,
                                               pose);
      init_odom_publisher_.publish(pose);

      // process keyframe
      if (IsKeyframe(img_time, T_WORLD_BASELINK)) {
        // update keyframe info
        bs_models::camera_to_camera::Keyframe kf(img_time,
                                                 image_buffer_.front());
        keyframes_.push_back(kf);
        added_since_kf_ = 0;
        // log pose info
        ROS_INFO("Estimated Keyframe Pose:");
        std::cout << T_WORLD_BASELINK << std::endl;
        // notify that a new keyframe is detected
        NotifyNewKeyframe(T_WORLD_CAMERA.value());
        // extend map
        ExtendMap(img_time);
        // publish oldest keyframe for global mapper
        PublishSlamChunk();
      } else {
        // compute relative pose to most recent kf
        Eigen::Matrix4d T_WORLD_CAMERA_curkf =
            visual_map_->GetCameraPose(keyframes_.back().Stamp()).value();
        Eigen::Matrix4d T_WORLD_BASELINK_curkf =
            T_WORLD_CAMERA_curkf * T_cam_baselink_;
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

void VisualInertialOdom::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  // push imu message onto buffer
  imu_buffer_.push(*msg);
  // get current image timestamp
  ros::Time img_time = image_buffer_.front().header.stamp;
  /**************************************************************************
   *          Add IMU messages to preintegrator or initializer              *
   **************************************************************************/
  while (imu_buffer_.front().header.stamp <= img_time && !imu_buffer_.empty()) {
    if (!initializer_->Initialized()) {
      initializer_->AddIMU(imu_buffer_.front());
    } else {
      imu_preint_->AddToBuffer(imu_buffer_.front());
    }
    imu_buffer_.pop();
  }
}

void VisualInertialOdom::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  visual_map_->UpdateGraph(graph);
}

void VisualInertialOdom::onStop() {}

void VisualInertialOdom::SendInitializationGraph(
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
  keyframe_header.frame_id = global_params_.baselink_frame;
  keyframe_header.seq = keyframes_.back().SequenceNumber();
  new_keyframe_publisher_.publish(keyframe_header);
  // publish landmarks
  PublishLandmarkIDs(new_landmarks);
}

bool VisualInertialOdom::IsKeyframe(const ros::Time& img_time,
                                    const Eigen::Matrix4d& T_WORLD_BASELINK) {
  Eigen::Matrix4d T_WORLD_curkf =
      visual_map_->GetBaselinkPose(keyframes_.back().Stamp()).value();
  bool is_keyframe = false;
  if ((img_time - keyframes_.back().Stamp()).toSec() >=
          camera_params_.keyframe_min_time_in_seconds &&
      beam::PassedMotionThreshold(T_WORLD_curkf, T_WORLD_BASELINK, 0.0, 0.05,
                                  true, true, false)) {
    ROS_INFO("New keyframe chosen at: %f", img_time.toSec());
    is_keyframe = true;
  } else if (added_since_kf_ == (camera_params_.window_size - 1)) {
    is_keyframe = true;
  } else {
    is_keyframe = false;
  }
  return is_keyframe;
}

void VisualInertialOdom::ExtendMap(const ros::Time& img_time) {
  // get current and previous keyframe timestamps
  ros::Time prev_kf_time = (keyframes_[keyframes_.size() - 2]).Stamp();
  ros::Time cur_kf_time = keyframes_.back().Stamp();
  std::vector<uint64_t> new_landmarks;

  // make transaction
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(cur_kf_time);

  // match against current submap and add fixed landmarks
  std::map<uint64_t, Eigen::Vector3d> matched_points =
      bs_models::camera_to_camera::MatchFrameToCurrentSubmap(
          tracker_, visual_map_, cam_model_, img_time);
  for (auto& it : matched_points) {
    visual_map_->AddFixedLandmark(it.second, it.first, transaction);
  }

  // add visual constraints
  std::vector<uint64_t> landmarks = tracker_->GetLandmarkIDsInImage(img_time);
  for (auto& id : landmarks) {
    // add constraints to triangulated ids
    if (visual_map_->GetLandmark(id) || visual_map_->GetFixedLandmark(id)) {
      visual_map_->AddConstraint(cur_kf_time, id,
                                 tracker_->Get(cur_kf_time, id), transaction);
    } else {
      // triangulate new point and add constraints
      Eigen::Vector2d pixel_prv_kf, pixel_cur_kf;
      Eigen::Matrix4d T_cam_world_prv_kf, T_cam_world_cur_kf;
      try {
        // get measurements
        pixel_prv_kf = tracker_->Get(prev_kf_time, id);
        pixel_cur_kf = tracker_->Get(cur_kf_time, id);
        T_cam_world_prv_kf =
            visual_map_->GetCameraPose(prev_kf_time).value().inverse();
        T_cam_world_cur_kf =
            visual_map_->GetCameraPose(cur_kf_time).value().inverse();
        // triangulate point
        Eigen::Vector2i pixel_prv_kf_i = pixel_prv_kf.cast<int>();
        Eigen::Vector2i pixel_cur_kf_i = pixel_cur_kf.cast<int>();
        beam::opt<Eigen::Vector3d> point =
            beam_cv::Triangulation::TriangulatePoint(
                cam_model_, cam_model_, T_cam_world_prv_kf, T_cam_world_cur_kf,
                pixel_prv_kf_i, pixel_cur_kf_i);
        // add landmark and constraints to map
        if (point.has_value()) {
          new_landmarks.push_back(id);
          visual_map_->AddLandmark(point.value(), id, transaction);
          visual_map_->AddConstraint(prev_kf_time, id, pixel_prv_kf,
                                     transaction);
          visual_map_->AddConstraint(cur_kf_time, id, pixel_cur_kf,
                                     transaction);
        }
      } catch (const std::out_of_range& oor) {}
    }
  }

  ROS_INFO("Added %zu new landmarks.", new_landmarks.size());
  // add inertial constraint
  // AddInertialConstraint(img_time, transaction);
  // send transaction to graph
  sendTransaction(transaction);
  // publish new landmarks
  PublishLandmarkIDs(new_landmarks);
}

void VisualInertialOdom::AddInertialConstraint(
    fuse_core::Transaction::SharedPtr transaction) {
  ros::Time cur_kf_time = keyframes_.back().Stamp();
  // get inertial constraint transaction
  fuse_core::Transaction::SharedPtr inertial_transaction =
      imu_preint_->RegisterNewImuPreintegratedFactor(
          cur_kf_time, visual_map_->GetOrientation(cur_kf_time),
          visual_map_->GetPosition(cur_kf_time));
  // merge with existing transaction
  transaction->merge(*inertial_transaction);
}

void VisualInertialOdom::NotifyNewKeyframe(
    const Eigen::Matrix4d& T_WORLD_CAMERA) {
  // send keyframe pose to graph
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(keyframes_.back().Stamp());
  visual_map_->AddPose(T_WORLD_CAMERA, keyframes_.back().Stamp(), transaction);
  sendTransaction(transaction);

  // build header message and publish for lidar slam
  std_msgs::Header keyframe_header;
  keyframe_header.stamp = keyframes_.back().Stamp();
  keyframe_header.frame_id = global_params_.baselink_frame;
  keyframe_header.seq = keyframes_.back().SequenceNumber();
  new_keyframe_publisher_.publish(keyframe_header);

  // make and publish reloc request
  RelocRequestMsg reloc_msg;
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

void VisualInertialOdom::PublishSlamChunk() {
  // this just makes sure the visual map has the most recent variables
  for (auto& kf : keyframes_) { visual_map_->GetCameraPose(kf.Stamp()); }
  // only once keyframes reaches the max window size, publish the keyframe
  if (keyframes_.size() == camera_params_.keyframe_window_size - 1) {
    // remove keyframe placeholder if first keyframe
    if (keyframes_.front().Stamp() == ros::Time(0)) { keyframes_.pop_front(); }
    // build slam chunk
    SlamChunkMsg slam_chunk;
    // stamp
    ros::Time kf_to_publish = keyframes_.front().Stamp();
    slam_chunk.stamp = kf_to_publish;
    // keyframe pose
    Eigen::Matrix4d T_WORLD_BASELINK =
        visual_map_->GetCameraPose(kf_to_publish).value() * T_cam_baselink_;
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
    camera_measurement.descriptor_type = 0;
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
      std::vector<float> descriptor_v;
      descriptor_v.assign((float*)descriptor.datastart,
                          (float*)descriptor.dataend);
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

void VisualInertialOdom::PublishLandmarkIDs(const std::vector<uint64_t>& ids) {
  std_msgs::UInt64MultiArray landmark_msg;
  for (auto& id : ids) { landmark_msg.data.push_back(id); }
  landmark_publisher_.publish(landmark_msg);
}

}} // namespace bs_models::camera_to_camera
