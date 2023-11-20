#include <bs_models/graph_publisher.h>

#include <filesystem>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/descriptors/Descriptor.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/time.h>

#include <bs_common/conversions.h>
#include <bs_common/graph_access.h>
#include <bs_common/utils.h>
#include <bs_common/visualization.h>
#include <bs_models/graph_visualization/helpers.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::GraphPublisher, fuse_core::SensorModel);

const std::string k_visual_measurements_topic{
    "/feature_tracker/visual_measurements"};
const std::string k_graph_path_topic{"poses"};
const std::string k_graph_odom_topic{"odom"};
const std::string k_cam_landmarks_topic{"camera_landmarks"};
const std::string k_cam_keypoints_image_topic{"tracked_image"};

namespace bs_models {

GraphPublisher::GraphPublisher()
    : fuse_core::AsyncSensorModel(1),
      throttled_measurement_callback_(
          std::bind(&GraphPublisher::processCameraMeasurements, this,
                    std::placeholders::_1)) {}

void GraphPublisher::onInit() {
  landmark_container_ = std::make_shared<beam_containers::LandmarkContainer>();
  bs_parameters::models::CalibrationParams calibration_params_;
  calibration_params_.loadFromROS();
  cam_model_ = beam_calibration::CameraModel::Create(
      calibration_params_.cam_intrinsics_path);
}

void GraphPublisher::onStart() {
  feature_track_subscriber_ =
      private_node_handle_.subscribe<bs_common::CameraMeasurementMsg>(
          k_visual_measurements_topic, 100,
          &ThrottledMeasurementCallback::callback,
          &throttled_measurement_callback_,
          ros::TransportHints().tcpNoDelay(false));

  // setup publishers
  graph_path_publisher_.publisher =
      private_node_handle_.advertise<nav_msgs::Path>(k_graph_path_topic, 10);
  graph_odom_publisher_.publisher =
      private_node_handle_.advertise<nav_msgs::Odometry>(k_graph_odom_topic,
                                                         10);
  camera_landmarks_publisher_.publisher =
      private_node_handle_.advertise<sensor_msgs::PointCloud2>(
          k_cam_landmarks_topic, 10);
  image_publisher_.publisher =
      private_node_handle_.advertise<sensor_msgs::Image>(
          k_cam_keypoints_image_topic, 10);
}

void GraphPublisher::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) {
  current_time_ = ros::Time::now();
  PublishPoses(graph_msg);
  PublishCameraLandmarks(graph_msg);
}

void GraphPublisher::PublishPoses(fuse_core::Graph::ConstSharedPtr graph_msg) {
  // publish path each time
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = extrinsics_.GetWorldFrameId();
  path_msg.header.seq = graph_path_publisher_.counter;
  std::map<ros::Time, Eigen::Matrix4d> poses =
      bs_common::GetGraphPoses(*graph_msg);
  int counter = 0;
  for (const auto& [stamp, T_World_Baselink] : poses) {
    geometry_msgs::PoseStamped pose_stamped;
    bs_common::EigenTransformToPoseStamped(T_World_Baselink, stamp, counter++,
                                           extrinsics_.GetBaselinkFrameId(),
                                           pose_stamped);
    path_msg.poses.push_back(pose_stamped);
  }
  graph_path_publisher_.publisher.publish(path_msg);
  graph_path_publisher_.counter++;

  // publish odom for poses that have been marginalized out, i.e., they don't
  // exist in the last poses
  if (poses_last_.empty()) {
    poses_last_ = poses;
    return;
  }

  // publish marginalized poses as odom
  for (const auto& [stamp, T_World_Baselink] : poses_last_) {
    auto pose_it = poses.find(stamp);
    // if we find the pose, then don't publish yet
    if (pose_it != poses.end()) { continue; }
    // otherwise, it has been marginalized so let's publish it
    nav_msgs::Odometry odom_msg;
    bs_common::EigenTransformToOdometryMsg(
        T_World_Baselink, stamp, graph_odom_publisher_.counter,
        extrinsics_.GetWorldFrameId(), extrinsics_.GetBaselinkFrameId(),
        odom_msg);
    graph_odom_publisher_.publisher.publish(odom_msg);
    graph_odom_publisher_.counter++;
  }
  poses_last_ = poses;
}

void GraphPublisher::PublishCameraLandmarks(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud =
      graph_visualization::GetGraphCameraLandmarksAsCloud(*graph_msg);
  PublishCloud<pcl::PointXYZRGBL>(camera_landmarks_publisher_, cloud);

  // get all timestamps in the graph
  auto timestamps = bs_common::CurrentTimestamps(*graph_msg);

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
    if (image_out.type() == CV_8UC1) {
      cv::Mat dst;
      cv::cvtColor(image_out, dst, cv::COLOR_GRAY2BGR);
      image_out = dst;
    }

    // get pose
    const auto position = bs_common::GetPosition(*graph_msg, timestamp);
    const auto orientation = bs_common::GetOrientation(*graph_msg, timestamp);
    Eigen::Matrix4d T_WORLD_BASELINK =
        bs_common::FusePoseToEigenTransform(*position, *orientation);

    // get landmarks in image
    auto lm_ids = landmark_container_->GetLandmarkIDsInImage(timestamp);
    for (const auto id : lm_ids) {
      Eigen::Vector2d pixel = landmark_container_->GetValue(timestamp, id);
      cv::Point m(pixel[0], pixel[1]);
      const auto lm_variable = bs_common::GetLandmark(*graph_msg, id);
      const auto idp_lm_variable =
          bs_common::GetInverseDepthLandmark(*graph_msg, id);
      if (lm_variable || idp_lm_variable) {
        Eigen::Vector3d camera_t_point;
        if (lm_variable) {
          camera_t_point =
              (T_CAM_BASELINK * beam::InvertTransform(T_WORLD_BASELINK) *
               lm_variable->point().homogeneous())
                  .hnormalized();
        } else if (idp_lm_variable) {
          Eigen::Vector3d anchor_t_point = idp_lm_variable->camera_t_point();
          const auto p = bs_common::GetPosition(*graph_msg,
                                                idp_lm_variable->anchorStamp());
          const auto o = bs_common::GetOrientation(
              *graph_msg, idp_lm_variable->anchorStamp());
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

    sensor_msgs::Image out_msg = beam_cv::OpenCVConversions::MatToRosImg(
        image_out, img_msg.header, "bgr8");
    image_publisher_.publisher.publish(out_msg);
    image_publisher_.counter++;

    image_buffer_.clear();
    landmark_container_->clear();
    break;
  }
}

void GraphPublisher::processCameraMeasurements(
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

} // namespace bs_models