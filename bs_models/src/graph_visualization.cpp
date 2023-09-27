#include <bs_models/graph_visualization.h>

#include <filesystem>

#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_constraints/relative_constraint.h>
#include <fuse_constraints/relative_pose_3d_stamped_constraint.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/descriptors/Descriptor.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/time.h>

#include <bs_common/conversions.h>
#include <bs_common/utils.h>
#include <bs_common/visualization.h>
#include <bs_constraints/global/absolute_pose_3d_constraint.h>
#include <bs_constraints/global/gravity_alignment_stamped_constraint.h>
#include <bs_constraints/inertial/absolute_imu_state_3d_stamped_constraint.h>
#include <bs_constraints/inertial/relative_imu_state_3d_stamped_constraint.h>
#include <bs_constraints/relative_pose/relative_pose_3d_stamped_with_extrinsics_constraint.h>
#include <bs_constraints/visual/euclidean_reprojection_constraint.h>
#include <bs_models/graph_visualization/helpers.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::GraphVisualization, fuse_core::SensorModel);

namespace bs_models {

using namespace graph_visualization;

GraphVisualization::GraphVisualization()
    : fuse_core::AsyncSensorModel(1),
      throttled_measurement_callback_(
          std::bind(&GraphVisualization::processMeasurements, this,
                    std::placeholders::_1)) {}

void GraphVisualization::onInit() {
  // Read settings from the parameter sever
  params_.loadFromROS(private_node_handle_);
  if (!params_.save_path.empty()) {
    if (!std::filesystem::exists(params_.save_path)) {
      BEAM_ERROR("Invalid save path, path must exist: {}", params_.save_path);
      throw std::runtime_error{"invalid save path"};
    }
    std::string time_now =
        beam::ConvertTimeToDate(std::chrono::system_clock::now());
    save_path_ = beam::CombinePaths(params_.save_path, time_now);
    std::filesystem::create_directory(save_path_);
  }

  landmark_container_ = std::make_shared<beam_containers::LandmarkContainer>();
  bs_parameters::models::CalibrationParams calibration_params_;
  calibration_params_.loadFromROS();
  cam_model_ = beam_calibration::CameraModel::Create(
      calibration_params_.cam_intrinsics_path);
}

void GraphVisualization::onStart() {
  feature_track_subscriber_ =
      private_node_handle_.subscribe<bs_common::CameraMeasurementMsg>(
          "/local_mapper/visual_feature_tracker/visual_measurements", 100,
          &ThrottledMeasurementCallback::callback,
          &throttled_measurement_callback_,
          ros::TransportHints().tcpNoDelay(false));

  // setup publishers
  if (params_.publish) {
    poses_publisher_.publisher =
        private_node_handle_.advertise<sensor_msgs::PointCloud2>("graph_poses",
                                                                 10);
    lidar_relative_pose_constraints_publisher_.publisher =
        private_node_handle_.advertise<sensor_msgs::PointCloud2>(
            "lidar_relative_pose_constraints", 10);
    relative_imu_constraints_publisher_.publisher =
        private_node_handle_.advertise<sensor_msgs::PointCloud2>(
            "imu_relative_constraints", 10);
    gravity_constraints_publisher_.publisher =
        private_node_handle_.advertise<sensor_msgs::PointCloud2>(
            "gravity_alignment_constraints", 10);
    camera_landmarks_publisher_.publisher =
        private_node_handle_.advertise<sensor_msgs::PointCloud2>(
            "camera_landmarks", 10);
    imu_biases_publisher_gx_ =
        private_node_handle_.advertise<std_msgs::Float32>("biases/gx", 10);
    imu_biases_publisher_gy_ =
        private_node_handle_.advertise<std_msgs::Float32>("biases/gy", 10);
    imu_biases_publisher_gz_ =
        private_node_handle_.advertise<std_msgs::Float32>("biases/gz", 10);
    imu_biases_publisher_ax_ =
        private_node_handle_.advertise<std_msgs::Float32>("biases/ax", 10);
    imu_biases_publisher_ay_ =
        private_node_handle_.advertise<std_msgs::Float32>("biases/ay", 10);
    imu_biases_publisher_az_ =
        private_node_handle_.advertise<std_msgs::Float32>("biases/az", 10);
    image_publisher_ =
        private_node_handle_.advertise<sensor_msgs::Image>("tracked_image", 10);
  }
}

void GraphVisualization::onGraphUpdate(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  current_time_ = ros::Time::now();
  VisualizePoses(graph_msg);
  VisualizeLidarRelativePoseConstraints(graph_msg);
  VisualizeImuRelativeConstraints(graph_msg);
  VisualizeImuBiases(graph_msg);
  VisualizeImuGravityConstraints(graph_msg);
  VisualizeCameraLandmarks(graph_msg);
  ValidateGraphPriors(graph_msg);
  ValidateGraphConnectivity(graph_msg);
}

void GraphVisualization::VisualizePoses(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud =
      bs_common::GetGraphPosesAsCloud(*graph_msg);
  PublishCloud<pcl::PointXYZRGBL>(poses_publisher_, cloud);
  SaveCloud<pcl::PointXYZRGBL>(
      save_path_, std::to_string(current_time_.toSec()) + "_graph_poses",
      cloud);
}

void GraphVisualization::VisualizeLidarRelativePoseConstraints(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud =
      bs_common::GetGraphRelativePoseConstraintsAsCloud(*graph_msg,
                                                        "LidarOdometry::");
  if (cloud.empty()) {
    cloud = GetGraphRelativePoseWithExtrinsicsConstraintsAsCloud(
        *graph_msg, "LidarOdometry::");
  }

  PublishCloud<pcl::PointXYZRGBL>(lidar_relative_pose_constraints_publisher_,
                                  cloud);
  SaveCloud<pcl::PointXYZRGBL>(save_path_,
                               std::to_string(current_time_.toSec()) +
                                   "_lidar_relative_pose_constraints",
                               cloud);
}

void GraphVisualization::VisualizeImuRelativeConstraints(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud =
      GetGraphRelativeImuConstraintsAsCloud(*graph_msg, point_spacing_,
                                            frame_size_);
  PublishCloud<pcl::PointXYZRGBL>(relative_imu_constraints_publisher_, cloud);
  SaveCloud<pcl::PointXYZRGBL>(save_path_,
                               std::to_string(current_time_.toSec()) +
                                   "_relative_imu_constraints",
                               cloud);
}

void GraphVisualization::VisualizeImuBiases(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  // load IMU biases
  std::map<int64_t, bs_common::ImuBiases> biases_in_graph =
      bs_common::GetImuBiasesFromGraph(*graph_msg);
  if (biases_in_graph.empty()) { return; }

  // publish most recent
  const bs_common::ImuBiases& biases_recent = biases_in_graph.rbegin()->second;
  std_msgs::Float32 ax, ay, az, gx, gy, gz;
  ax.data = biases_recent.a_x;
  ay.data = biases_recent.a_y;
  az.data = biases_recent.a_z;
  gx.data = biases_recent.g_x;
  gy.data = biases_recent.g_y;
  gz.data = biases_recent.g_z;
  imu_biases_publisher_gx_.publish(gx);
  imu_biases_publisher_gy_.publish(gy);
  imu_biases_publisher_gz_.publish(gz);
  imu_biases_publisher_ax_.publish(ax);
  imu_biases_publisher_ay_.publish(ay);
  imu_biases_publisher_az_.publish(az);

  // save window of biases
  SaveImuBiases(biases_in_graph, save_path_,
                std::to_string(current_time_.toSec()) + "_imu_biases");
}

void GraphVisualization::VisualizeImuGravityConstraints(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud = GetGraphGravityConstraintsAsCloud(
      *graph_msg, point_spacing_, frame_size_, g_length_);
  PublishCloud<pcl::PointXYZRGBL>(gravity_constraints_publisher_, cloud);
  SaveCloud<pcl::PointXYZRGBL>(
      save_path_,
      std::to_string(current_time_.toSec()) + "_gravity_constraints", cloud);
}

void GraphVisualization::VisualizeCameraLandmarks(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud =
      GetGraphCameraLandmarksAsCloud(*graph_msg);
  PublishCloud<pcl::PointXYZRGBL>(camera_landmarks_publisher_, cloud);
  SaveCloud<pcl::PointXYZRGBL>(
      save_path_, std::to_string(current_time_.toSec()) + "_camera_landmarks",
      cloud);

  // get all timestamps in the graph
  auto timestamps = bs_common::CurrentTimestamps(graph_msg);

  Eigen::Matrix4d T_CAM_BASELINK;
  extrinsics_.GetT_CAMERA_BASELINK(T_CAM_BASELINK);
  Eigen::Matrix4d T_BASELINK_CAM = beam::InvertTransform(T_CAM_BASELINK);

  static cv::Scalar green(0, 255, 0);
  static cv::Scalar blue(255, 0, 0);
  static cv::Scalar red(0, 0, 255);
  static cv::Scalar yellow(0, 255, 255);

  // get most recent timestamp in graph that has associated image
  for (auto itr = timestamps.rbegin(); itr != timestamps.rend(); itr++) {
    const auto timestamp = *itr;
    const auto nsec = timestamp.toNSec();
    if (image_buffer_.find(nsec) == image_buffer_.end()) { continue; }

    const auto img_msg = image_buffer_[nsec];
    cv::Mat image_out = beam_cv::OpenCVConversions::RosImgToMat(img_msg);

    // get pose
    const auto position = bs_common::GetPosition(graph_msg, timestamp);
    const auto orientation = bs_common::GetOrientation(graph_msg, timestamp);
    Eigen::Matrix4d T_WORLD_BASELINK =
        bs_common::FusePoseToEigenTransform(*position, *orientation);

    // get landmarks in image
    auto lm_ids = landmark_container_->GetLandmarkIDsInImage(timestamp);
    for (const auto id : lm_ids) {
      Eigen::Vector2d pixel = landmark_container_->GetValue(timestamp, id);
      cv::Point m(pixel[0], pixel[1]);
      const auto lm_variable = bs_common::GetLandmark(graph_msg, id);
      const auto idp_lm_variable =
          bs_common::GetInverseDepthLandmark(graph_msg, id);
      if (lm_variable || idp_lm_variable) {
        Eigen::Vector3d camera_t_point;
        if (lm_variable) {
          camera_t_point =
              (T_CAM_BASELINK * beam::InvertTransform(T_WORLD_BASELINK) *
               lm_variable->point().homogeneous())
                  .hnormalized();
        } else if (idp_lm_variable) {
          Eigen::Vector3d anchor_t_point = idp_lm_variable->camera_t_point();
          const auto p =
              bs_common::GetPosition(graph_msg, idp_lm_variable->anchorStamp());
          const auto o = bs_common::GetOrientation(
              graph_msg, idp_lm_variable->anchorStamp());
          if (!o || !p) { continue; }
          Eigen::Matrix4d T_WORLD_CAMERAmeasurement =
              T_WORLD_BASELINK * T_BASELINK_CAM;
          Eigen::Matrix4d T_WORLD_CAMERAanchor =
              bs_common::FusePoseToEigenTransform(*p, *o) * T_BASELINK_CAM;
          Eigen::Matrix4d T_CAMERAmeasurement_CAMERAanchor =
              beam::InvertTransform(T_WORLD_CAMERAmeasurement) *
              T_WORLD_CAMERAanchor;
          camera_t_point =
              (T_CAMERAmeasurement_CAMERAanchor * anchor_t_point.homogeneous())
                  .hnormalized();
        }

        Eigen::Vector2d projected;
        if (cam_model_->ProjectPoint(camera_t_point, projected)) {
          // draw pixel-point pair in image
          cv::Point e(projected[0], projected[1]);
          cv::circle(image_out, m, keypoints_circle_radius_, green,
                     keypoints_line_thickness_);
          cv::circle(image_out, e, keypoints_circle_radius_, blue,
                     keypoints_line_thickness_);
          cv::line(image_out, m, e, green, keypoints_circle_radius_, 8);
        }
      } else {
        // draw just the pixel in yellow
        cv::circle(image_out, m, keypoints_circle_radius_, yellow,
                   keypoints_line_thickness_);
      }
    }

    // todo: this is commented out for now since we dont do any map matching yet
    // // get landmarks not tracked in image
    // std::sort(lm_ids.begin(), lm_ids.end());
    // const auto graph_lms_set = bs_common::CurrentLandmarkIDs(graph_msg);
    // std::vector<uint64_t> graph_lms(graph_lms_set.size());
    // std::copy(graph_lms_set.begin(), graph_lms_set.end(), graph_lms.begin());
    // std::vector<uint64_t> non_tracked_landmarks;
    // std::set_difference(
    //     graph_lms.begin(), graph_lms.end(), lm_ids.begin(), lm_ids.end(),
    //     std::inserter(non_tracked_landmarks, non_tracked_landmarks.begin()));
    // for (const auto id : non_tracked_landmarks) {
    //   const auto lm_variable = bs_common::GetLandmark(graph_msg, id);
    //   if (lm_variable) {
    //     Eigen::Vector3d point =
    //         (T_CAM_BASELINK * beam::InvertTransform(T_WORLD_BASELINK) *
    //          lm_variable->point().homogeneous())
    //             .hnormalized();
    //     Eigen::Vector2d projected;
    //     if (cam_model_->ProjectPoint(point, projected)) {
    //       // draw projected point, with no track as red
    //       cv::Point e(projected[0], projected[1]);
    //       cv::circle(image_out, e, 1, red, 2);
    //     }
    //   }
    // }

    sensor_msgs::Image out_msg = beam_cv::OpenCVConversions::MatToRosImg(
        image_out, img_msg.header, "bgr8");
    image_publisher_.publish(out_msg);

    image_buffer_.clear();
    landmark_container_->clear();
    break;
  }
}

void GraphVisualization::processMeasurements(
    const bs_common::CameraMeasurementMsg::ConstPtr& msg) {
  image_buffer_[msg->header.stamp.toNSec()] = msg->image;
  // check that message hasnt already been added to container
  const auto times = landmark_container_->GetMeasurementTimes();
  if (times.find(msg->header.stamp) != times.end()) { return; }

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
  }
}

void GraphVisualization::ValidateGraphPriors(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  if (!params_.validate_graph) { return; }

  const auto constraints = graph_msg->getConstraints();
  for (auto it = constraints.begin(); it != constraints.end(); it++) {
    if (it->type() == "bs_constraints::AbsoluteImuState3DStampedConstraint") {
      if (!absolute_imu_constraints_.empty()) {
        if (absolute_imu_constraints_.find(it->uuid()) ==
            absolute_imu_constraints_.end()) {
          ROS_ERROR_ONCE("Detected multiple absolute constraints of type: "
                         "bs_constraints::AbsoluteImuState3DStampedConstraint");
        }
        // else do nothing, this is the same one so it's ok
      } else {
        absolute_imu_constraints_.emplace(it->uuid());
      }
    } else if (it->type() == "bs_constraints::AbsolutePose3DConstraint") {
      auto c =
          dynamic_cast<const bs_constraints::AbsolutePose3DConstraint&>(*it);
      auto position = c.getInitialPosition();
      std::string T = "T_" + position.child() + "_" + position.parent();
      auto iter = extrinsics_constraints_.find(T);
      if (iter != extrinsics_constraints_.end()) {
        if (iter->second != it->uuid()) {
          ROS_ERROR_ONCE(
              "Detected multiple time-invariant pose priors representing "
              "the transform: %s",
              T.c_str());
        }
        // else do nothing, this is the same one so it's ok
      } else {
        extrinsics_constraints_.emplace(T, it->uuid());
      }
    } else if (it->type() ==
               "fuse_constraints::AbsolutePose3DStampedConstraint") {
      if (!absolute_pose_constraints_.empty()) {
        if (absolute_pose_constraints_.find(it->uuid()) ==
            absolute_pose_constraints_.end()) {
          ROS_ERROR_ONCE("Detected multiple absolute constraints of type: "
                         "fuse_constraints::AbsolutePose3DStampedConstraint");
        }
        // else do nothing, this is the same one so it's ok
      } else {
        absolute_pose_constraints_.emplace(it->uuid());
      }
    }
  }

  if (absolute_imu_constraints_.empty() && absolute_pose_constraints_.empty()) {
    ROS_ERROR_ONCE("No IMU absolute constraint found in graph");
  }

  if (!absolute_imu_constraints_.empty() &&
      !absolute_pose_constraints_.empty()) {
    ROS_ERROR_ONCE(
        "Detected an absolute stamped pose constraint, and an IMU "
        "pose prior constraint. These constrain the same variables!");
  }
}

void GraphVisualization::ValidateGraphConnectivity(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  if (!params_.validate_graph) { return; }

  // get all stamped positions
  VariableConnectivityType varConnectivity;
  const auto v_range = graph_msg->getVariables();
  for (auto it = v_range.begin(); it != v_range.end(); it++) {
    if (it->type() == "fuse_variables::Position3DStamped") {
      auto v = dynamic_cast<const fuse_variables::Position3DStamped&>(*it);
      varConnectivity.emplace(it->uuid(), VariableConnectivity(v.stamp()));
    }
  }
  if (varConnectivity.size() < 3) { return; }

  // add connectivity for all constraints attached to position variables
  const auto c_range = graph_msg->getConstraints();
  for (auto it = c_range.begin(); it != c_range.end(); it++) {
    if (it->type() == "bs_constraints::AbsoluteImuState3DStampedConstraint") {
      auto c = dynamic_cast<
          const bs_constraints::AbsoluteImuState3DStampedConstraint&>(*it);
      const auto& pos_uuid = c.variables().at(1);
      if (varConnectivity.find(pos_uuid) == varConnectivity.end()) {
        ROS_ERROR_ONCE("no position variable found for constraint: %s",
                       to_string(c.uuid()).c_str());
        continue;
      }
      varConnectivity.at(pos_uuid).absolute_constraints.emplace(
          c.uuid(), "bs_constraints::AbsoluteImuState3DStampedConstraint");
    } else if (it->type() ==
               "fuse_constraints::AbsolutePose3DStampedConstraint") {
      auto c = dynamic_cast<
          const fuse_constraints::AbsolutePose3DStampedConstraint&>(*it);
      const auto& pos_uuid = c.variables().at(0);
      if (varConnectivity.find(pos_uuid) == varConnectivity.end()) {
        ROS_ERROR_ONCE("no position variable found for constraint: %s",
                       to_string(c.uuid()).c_str());
        continue;
      }
      varConnectivity.at(pos_uuid).absolute_constraints.emplace(
          c.uuid(), "fuse_constraints::AbsolutePose3DStampedConstraint");
    } else if (it->type() == "bs_constraints::"
                             "RelativePose3DStampedWithExtrinsicsConstraint") {
      auto c = dynamic_cast<
          const bs_constraints::RelativePose3DStampedWithExtrinsicsConstraint&>(
          *it);
      const auto& pos_uuid1 = c.variables().at(0);
      if (varConnectivity.find(pos_uuid1) == varConnectivity.end()) {
        ROS_ERROR_ONCE(
            "no position variable found for first position in constraint: %s",
            to_string(c.uuid()).c_str());
      } else {
        varConnectivity.at(pos_uuid1).next_constraints.emplace(
            c.uuid(),
            "bs_constraints::RelativePose3DStampedWithExtrinsicsConstraint");
      }
      const auto& pos_uuid2 = c.variables().at(2);
      if (varConnectivity.find(pos_uuid2) == varConnectivity.end()) {
        ROS_ERROR_ONCE(
            "no position variable found for second position in constraint: %s",
            to_string(c.uuid()).c_str());
      } else {
        varConnectivity.at(pos_uuid2).previous_constraints.emplace(
            c.uuid(),
            "bs_constraints::RelativePose3DStampedWithExtrinsicsConstraint");
      }
    } else if (it->type() ==
               "fuse_constraints::RelativePose3DStampedConstraint") {
      auto c = dynamic_cast<
          const fuse_constraints::RelativePose3DStampedConstraint&>(*it);
      const auto& pos_uuid1 = c.variables().at(0);
      if (varConnectivity.find(pos_uuid1) == varConnectivity.end()) {
        ROS_ERROR_ONCE(
            "no position variable found for first position in constraint: %s",
            to_string(c.uuid()).c_str());
      } else {
        varConnectivity.at(pos_uuid1).next_constraints.emplace(
            c.uuid(), "fuse_constraints::RelativePose3DStampedConstraint");
      }
      const auto& pos_uuid2 = c.variables().at(2);
      if (varConnectivity.find(pos_uuid2) == varConnectivity.end()) {
        ROS_ERROR_ONCE(
            "no position variable found for second position in constraint: %s",
            to_string(c.uuid()).c_str());
      } else {
        varConnectivity.at(pos_uuid2).previous_constraints.emplace(
            c.uuid(), "fuse_constraints::RelativePose3DStampedConstraint");
      }
    } else if (it->type() ==
               "bs_constraints::RelativeImuState3DStampedConstraint") {
      auto c = dynamic_cast<
          const bs_constraints::RelativeImuState3DStampedConstraint&>(*it);
      const auto& pos_uuid1 = c.variables().at(1);
      if (varConnectivity.find(pos_uuid1) == varConnectivity.end()) {
        ROS_ERROR_ONCE(
            "no position variable found for first position in constraint: %s",
            to_string(c.uuid()).c_str());
      } else {
        varConnectivity.at(pos_uuid1).next_constraints.emplace(
            c.uuid(), "bs_constraints::RelativeImuState3DStampedConstraint");
      }
      const auto& pos_uuid2 = c.variables().at(6);
      if (varConnectivity.find(pos_uuid2) == varConnectivity.end()) {
        ROS_ERROR_ONCE(
            "no position variable found for second position in constraint: %s",
            to_string(c.uuid()).c_str());
      } else {
        varConnectivity.at(pos_uuid2).previous_constraints.emplace(
            c.uuid(), "bs_constraints::RelativeImuState3DStampedConstraint");
      }
    } else if (it->type() == "bs_constraints::AbsolutePose3DConstraint" ||
               it->type() == "fuse_constraints::MarginalConstraint" ||
               it->type() ==
                   "bs_constraints::EuclideanReprojectionConstraint" ||
               it->type() ==
                   "bs_constraints::InverseDepthReprojectionConstraintUnary" ||
               it->type() ==
                   "bs_constraints::InverseDepthReprojectionConstraint") {
      // skip all these types
    } else {
      ROS_WARN_ONCE("Unknown constraint type: %s", it->type().c_str());
    }
  }

  // go through connectivity map and check

  // check first position:
  auto first_uuid = GetFirstVariable(varConnectivity);
  const auto& first_conn = varConnectivity.at(first_uuid);
  if (!prior_found_on_first_ && first_conn.absolute_constraints.empty()) {
    ROS_ERROR_ONCE("First position (%s) does not have any priors",
                   to_string(first_uuid).c_str());
  } else {
    prior_found_on_first_ = true;
  }
  if (!first_conn.previous_constraints.empty()) {
    ROS_ERROR_ONCE(
        "First position (%s) has constraints against previous positions",
        to_string(first_uuid).c_str());
  }
  if (first_conn.next_constraints.empty()) {
    ROS_ERROR_ONCE(
        "Position (%s) has no constraints against subsequent positions",
        to_string(first_uuid).c_str());
  } else if (!HasImuConstraint(first_conn.next_constraints)) {
    ROS_ERROR_ONCE(
        "Position (%s) has no IMU constraints against subsequent positions",
        to_string(first_uuid).c_str());
  }

  // check last position
  auto last_uuid = GetLastVariable(varConnectivity);
  const auto& last_conn = varConnectivity.at(last_uuid);
  if (last_conn.previous_constraints.empty()) {
    ROS_ERROR_ONCE(
        "Position (%s) has no constraints against previous positions",
        to_string(last_uuid).c_str());
  } else if (!HasImuConstraint(last_conn.previous_constraints)) {
    ROS_ERROR_ONCE(
        "Position (%s) has no IMU constraints against previous positions",
        to_string(last_uuid).c_str());
  }

  // check all intermediate positions:
  for (const auto& [uuid, var_conn] : varConnectivity) {
    if (uuid == first_uuid || uuid == last_uuid) { continue; }
    if (!var_conn.absolute_constraints.empty()) {
      ROS_ERROR_ONCE(
          "intermediate position variable with uuid %s has absolute constraint "
          "with uuid: %s and type %s",
          to_string(uuid).c_str(),
          to_string(var_conn.absolute_constraints.begin()->first).c_str(),
          var_conn.absolute_constraints.begin()->second.c_str());
    }
    if (var_conn.previous_constraints.empty()) {
      ROS_ERROR_ONCE(
          "variable with uuid %s has no constraints to previous positions",
          to_string(uuid).c_str());
    } else if (!HasImuConstraint(var_conn.previous_constraints)) {
      ROS_ERROR_ONCE(
          "variable with uuid %s has no IMU constraints to previous positions",
          to_string(uuid).c_str());
    }
    if (var_conn.next_constraints.empty()) {
      ROS_ERROR_ONCE(
          "variable with uuid %s has no constraints to previous positions",
          to_string(uuid).c_str());
    } else if (!HasImuConstraint(var_conn.next_constraints)) {
      ROS_ERROR_ONCE(
          "variable with uuid %s has no IMU constraints to previous positions",
          to_string(uuid).c_str());
    }
  }
}

fuse_core::UUID GraphVisualization::GetFirstVariable(
    const VariableConnectivityType& connectivity) const {
  ros::Time first_time = connectivity.begin()->second.stamp;
  fuse_core::UUID first_uuid = connectivity.begin()->first;

  for (const auto& [uuid, conn] : connectivity) {
    if (conn.stamp < first_time) {
      first_time = conn.stamp;
      first_uuid = uuid;
    }
  }

  return first_uuid;
}

fuse_core::UUID GraphVisualization::GetLastVariable(
    const VariableConnectivityType& connectivity) const {
  ros::Time last_time = connectivity.begin()->second.stamp;
  fuse_core::UUID last_uuid = connectivity.begin()->first;

  for (const auto& [uuid, conn] : connectivity) {
    if (conn.stamp > last_time) {
      last_time = conn.stamp;
      last_uuid = uuid;
    }
  }

  return last_uuid;
}

bool GraphVisualization::HasImuConstraint(
    const ConstraintTypeMap& constraints) const {
  for (const auto& [id, type] : constraints) {
    if (type == "bs_constraints::RelativeImuState3DStampedConstraint") {
      return true;
    }
  }
  return false;
}

} // namespace bs_models