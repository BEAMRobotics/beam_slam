#include <bs_models/visual_odometry.h>

#include <bs_constraints/visual/euclidean_reprojection_constraint.h>
#include <fuse_core/transaction.h>
#include <fuse_variables/acceleration_linear_3d_stamped.h>
#include <fuse_variables/velocity_angular_3d_stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>

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
}

void VisualOdometry::onStart() {
  // setup subscribers
  measurement_subscriber_ =
      private_node_handle_.subscribe<CameraMeasurementMsg>(
          ros::names::resolve(
              "/local_mapper/visual_feature_tracker/visual_measurements"),
          100, &ThrottledMeasurementCallback::callback,
          &throttled_measurement_callback_,
          ros::TransportHints().tcpNoDelay(false));

  // setup publishers
  odometry_publisher_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odometry", 100);
  keyframe_publisher_ =
      private_node_handle_.advertise<geometry_msgs::PoseStamped>("pose", 100);
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

    // buffer frame if localization fails (only if frame init isnt caught up)
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

  // // add acceleration and velocity
  // auto lin_acc =
  //     fuse_variables::AccelerationLinear3DStamped::make_shared(timestamp);
  // auto ang_vel =
  //     fuse_variables::VelocityAngular3DStamped::make_shared(timestamp);
  // lin_acc->array() = {0.0, 0.0, 0.0};
  // ang_vel->array() = {0.0, 0.0, 0.0};
  // transaction->addVariable(lin_acc);
  // transaction->addVariable(ang_vel);

  previous_frame_ = timestamp;
  sendTransaction(transaction);

  if (IsKeyframe(timestamp, T_WORLD_BASELINK)) {
    ROS_INFO_STREAM("VisualOdometry: New keyframe detected at: " << timestamp);
    Keyframe kf(*msg);
    keyframes_.push_back(kf);
    ExtendMap(timestamp, T_WORLD_BASELINK);

    // publish keyframe pose
    PublishPose(timestamp, T_WORLD_BASELINK);

    // publish reloc request at given rate
    if ((timestamp - previous_reloc_request_).toSec() >
        vo_params_.reloc_request_period) {
      ROS_INFO_STREAM("Publishing reloc request at: " << timestamp);
      previous_reloc_request_ = timestamp;
      PublishRelocRequest(kf);
    }
  }

  return true;
}

bool VisualOdometry::LocalizeFrame(const ros::Time& timestamp,
                                   Eigen::Matrix4d& T_WORLD_BASELINK) {
  const auto prev_frame_pose = visual_map_->GetBaselinkPose(previous_frame_);
  if (!prev_frame_pose.has_value()) {
    ROS_ERROR("Cannot retrieve previous keyframe pose.");
    throw std::runtime_error{"Cannot retrieve previous keyframe pose."};
  }
  const Eigen::Matrix4d T_WORLD_BASELINKprev = prev_frame_pose.value();

  // get estimate from imu
  Eigen::Matrix4d T_PREVFRAME_CURFRAME;
  if (!frame_initializer_->GetRelativePose(T_PREVFRAME_CURFRAME,
                                           previous_frame_, timestamp)) {
    ROS_WARN_STREAM("Unable to estimate pose from frame initializer, "
                    "buffering frame: "
                    << timestamp);
    return false;
  }
  const Eigen::Matrix4d T_WORLD_BASELINKcur =
      T_WORLD_BASELINKprev * T_PREVFRAME_CURFRAME;

  // get 2d-3d correspondences
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  GetPixelPointPairs(timestamp, pixels, points);

  // perform motion only BA to refine estimate
  if (pixels.size() >= 20) {
    Eigen::Matrix4d T_CAMERA_WORLD_est = beam::InvertTransform(
        T_WORLD_BASELINKcur * beam::InvertTransform(T_cam_baselink_));
    // add a prior on the initial imu estimate
    Eigen::Matrix<double, 6, 6> prior =
        0.0001 * Eigen::Matrix<double, 6, 6>::Identity();
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
  }

  return true;
}

void VisualOdometry::ExtendMap(const ros::Time& timestamp,
                               const Eigen::Matrix4d& T_WORLD_BASELINK) {
  // create transaction for this keyframe
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(timestamp);

  // add prior if using a frame initializer
  if (frame_initializer_ && vo_params_.prior_information_weight != 0) {
    visual_map_->AddPosePrior(timestamp, vo_params_.prior_covariance,
                              transaction);
  }

  // process each landmark
  const auto landmarks = landmark_container_->GetLandmarkIDsInImage(timestamp);
  auto process_landmark = [&](const auto& id) {
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
      for (const auto& kf : keyframes_) {
        const auto kf_stamp = kf.Stamp();
        try {
          Eigen::Vector2d pixel = landmark_container_->GetValue(kf_stamp, id);
          visual_map_->AddVisualConstraint(kf_stamp, id, pixel, transaction);
        } catch (const std::out_of_range& oor) { continue; }
      }
    }
  };
  std::for_each(landmarks.begin(), landmarks.end(), process_landmark);

  sendTransaction(transaction);
}

void VisualOdometry::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  ROS_INFO_STREAM_ONCE("VisualOdometry received initial graph.");

  // all timestamps in the new graph
  const auto timestamps = bs_common::CurrentTimestamps(graph);

  // publish marginalized keyframes as slam chunks
  if (is_initialized_) {
    while (!keyframes_.empty() &&
           timestamps.find(keyframes_.front().Stamp()) == timestamps.end()) {
      PublishSlamChunk(keyframes_.front());
      keyframes_.pop_front();
    }
  }

  // Update graph object in visual map
  visual_map_->UpdateGraph(graph);

  // create motion only local copy of graph
  local_graph_ = graph->clone();
  for (auto& var : local_graph_->getVariables()) {
    if (var.type() != "fuse_variables::Orientation3DStamped" &&
        var.type() != "fuse_variables::Position3DStamped") {
      local_graph_->holdVariable(var.uuid(), true);
    }
  }

  // do initial setup
  if (!is_initialized_) {
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
    std::map<uint64_t, CameraMeasurementMsg::ConstPtr> measurement_map;
    std::for_each(
        visual_measurement_buffer_.begin(), visual_measurement_buffer_.end(),
        [&](auto msg) { measurement_map[msg->header.stamp.toNSec()] = msg; });

    // add each measurement as a keyframe if its in the graph
    for (const auto& stamp : union_stamps) {
      const auto msg = measurement_map.at(stamp);
      Keyframe kf(*msg);
      keyframes_.push_back(kf);
      previous_frame_ = msg->header.stamp;
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

  const auto kf_time = keyframes_.back().Stamp();
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
  const double percent_tracked =
      num_correspondences / static_cast<double>(frame1_ids.size());

  if (avg_parallax > vo_params_.keyframe_parallax) {
    return true;
  } else if (percent_tracked <= 0.5) {
    return true;
  }
  return false;
}

void VisualOdometry::AddMeasurementsToContainer(
    const CameraMeasurementMsg::ConstPtr& msg) {
  // check that message hasnt already been added to container
  const auto times = landmark_container_->GetMeasurementTimes();
  if (times.find(msg->header.stamp) != times.end()) { return; }

  // put all measurements into landmark container
  beam::HighResolutionTimer timer;
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
  }
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
  if (T_cam_world_v.size() >= 5) {
    return beam_cv::Triangulation::TriangulatePoint(
        cam_model_, T_cam_world_v, pixels,
        vo_params_.max_triangulation_distance,
        vo_params_.max_triangulation_reprojection);
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
    fuse_variables::Point3DLandmark::SharedPtr lm =
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

void VisualOdometry::ComputeRelativeOdometry(
    const ros::Time& timestamp, const Eigen::Matrix4d& T_WORLD_BASELINKcur) {
  // todo: fix this, the odom pose is garbage, the normal pose is fine
  static uint64_t rel_odom_seq = 0;
  const auto prev_frame_pose = visual_map_->GetBaselinkPose(previous_frame_);
  const Eigen::Matrix4d T_WORLD_BASELINKprev = prev_frame_pose.value();
  const Eigen::Matrix4d T_PREVKF_CURFRAME =
      beam::InvertTransform(T_WORLD_BASELINKprev) * T_WORLD_BASELINKcur;
  const Eigen::Matrix4d T_ODOM_BASELINKcur =
      T_ODOM_BASELINKprev_ * T_PREVKF_CURFRAME;
  const auto odom_msg = bs_common::TransformToOdometryMessage(
      timestamp, rel_odom_seq++, extrinsics_.GetWorldFrameId(),
      extrinsics_.GetBaselinkFrameId(), T_WORLD_BASELINKcur);
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

void VisualOdometry::PublishPose(const ros::Time& timestamp,
                                 const Eigen::Matrix4d& T_WORLD_BASELINK) {
  static uint64_t kf_odom_seq = 0;
  geometry_msgs::PoseStamped msg;
  bs_common::EigenTransformToPoseStamped(T_WORLD_BASELINK, timestamp,
                                         kf_odom_seq++,
                                         extrinsics_.GetBaselinkFrameId(), msg);
  keyframe_publisher_.publish(msg);
}

/****************************************************/
/*                                                  */
/*                                                  */
/*              Local Graph Functions               */
/*                                                  */
/*                                                  */
/****************************************************/
// void VisualOdometry::AddLocalCameraPose(
//     const ros::Time& stamp, const Eigen::Matrix4d& T_WORLD_BASELINK) {
//   // transform pose into baselink coord space
//   Eigen::Quaterniond q;
//   Eigen::Vector3d p;
//   beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_BASELINK, q, p);

//   // construct position variable
//   fuse_variables::Position3DStamped::SharedPtr position =
//       fuse_variables::Position3DStamped::make_shared(stamp);
//   position->x() = p[0];
//   position->y() = p[1];
//   position->z() = p[2];
//   local_graph_->addVariable(position);

//   // add orientation variable
//   fuse_variables::Orientation3DStamped::SharedPtr orientation =
//       fuse_variables::Orientation3DStamped::make_shared(stamp);
//   orientation->w() = q.w();
//   orientation->x() = q.x();
//   orientation->y() = q.y();
//   orientation->z() = q.z();
//   local_graph_->addVariable(orientation);
// }

// bool VisualOdometry::AddLocalVisualConstraint(const ros::Time& stamp,
//                                               const uint64_t id,
//                                               const Eigen::Vector2d& pixel) {
//   auto position = fuse_variables::Position3DStamped::make_shared();
//   auto position_uuid =
//       fuse_core::uuid::generate(position->type(), stamp,
//       fuse_core::uuid::NIL);
//   *position = dynamic_cast<const fuse_variables::Position3DStamped&>(
//       local_graph_->getVariable(position_uuid));

//   auto orientation = fuse_variables::Orientation3DStamped::make_shared();
//   auto orientation_uuid = fuse_core::uuid::generate(orientation->type(),
//   stamp,
//                                                     fuse_core::uuid::NIL);
//   *orientation = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
//       local_graph_->getVariable(orientation_uuid));

//   auto landmark = fuse_variables::Point3DLandmark::make_shared();
//   auto landmark_uuid = fuse_core::uuid::generate(landmark->type(), id);
//   *landmark = dynamic_cast<const fuse_variables::Point3DLandmark&>(
//       local_graph_->getVariable(landmark_uuid));

//   // rectify pixel
//   Eigen::Vector2i rectified_pixel;
//   if (!cam_model_->UndistortPixel(pixel.cast<int>(), rectified_pixel)) {
//     return false;
//   }
//   Eigen::Vector2d measurement = rectified_pixel.cast<double>();

//   if (!position || !orientation) { return false; }
//   try {
//     if (landmark) {
//       auto vis_constraint =
//           std::make_shared<bs_constraints::EuclideanReprojectionConstraint>(
//               "VO", *orientation, *position, *landmark, T_cam_baselink_,
//               cam_intrinsic_matrix_, measurement,
//               vo_params_.reprojection_information_weight);
//       vis_constraint->loss(vo_params_.reprojection_loss);
//       local_graph_->addConstraint(vis_constraint);
//       return true;
//     }
//   } catch (const std::logic_error& le) {}

//   return false;
// }

// void VisualOdometry::AddLocalPosePrior(
//     const ros::Time& stamp, const Eigen::Matrix<double, 6, 6>& covariance) {
//   auto position = fuse_variables::Position3DStamped::make_shared();
//   auto position_uuid =
//       fuse_core::uuid::generate(position->type(), stamp,
//       fuse_core::uuid::NIL);
//   *position = dynamic_cast<const fuse_variables::Position3DStamped&>(
//       local_graph_->getVariable(position_uuid));

//   auto orientation = fuse_variables::Orientation3DStamped::make_shared();
//   auto orientation_uuid = fuse_core::uuid::generate(orientation->type(),
//   stamp,
//                                                     fuse_core::uuid::NIL);
//   *orientation = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
//       local_graph_->getVariable(orientation_uuid));

//   if (position && orientation) {
//     fuse_core::Vector7d mean;
//     mean << position->x(), position->y(), position->z(), orientation->w(),
//         orientation->x(), orientation->y(), orientation->z();

//     auto prior =
//         std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
//             "PRIOR", *position, *orientation, mean, covariance);
//     local_graph_->addConstraint(prior);
//   }
// }

// beam::opt<Eigen::Matrix4d>
//     VisualOdometry::GetLocalBaselinkPose(const ros::Time& stamp) {
//   auto position = fuse_variables::Position3DStamped::make_shared();
//   auto position_uuid =
//       fuse_core::uuid::generate(position->type(), stamp,
//       fuse_core::uuid::NIL);
//   *position = dynamic_cast<const fuse_variables::Position3DStamped&>(
//       local_graph_->getVariable(position_uuid));

//   auto orientation = fuse_variables::Orientation3DStamped::make_shared();
//   auto orientation_uuid = fuse_core::uuid::generate(orientation->type(),
//   stamp,
//                                                     fuse_core::uuid::NIL);
//   *orientation = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
//       local_graph_->getVariable(orientation_uuid));
//   if (position && orientation) {
//     Eigen::Vector3d p(position->data());
//     Eigen::Quaterniond q(orientation->w(), orientation->x(),
//     orientation->y(),
//                          orientation->z());
//     Eigen::Matrix4d T_WORLD_BASELINK;
//     beam::QuaternionAndTranslationToTransformMatrix(q, p, T_WORLD_BASELINK);
//     return T_WORLD_BASELINK;
//   } else {
//     return {};
//   }
// }

} // namespace bs_models
