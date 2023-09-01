#include <bs_models/graph_visualization.h>

#include <boost/filesystem.hpp>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/descriptors/Descriptor.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/time.h>

#include <bs_common/conversions.h>
#include <bs_common/utils.h>
#include <bs_common/visualization.h>
#include <bs_constraints/global/gravity_alignment_stamped_constraint.h>
#include <bs_constraints/inertial/relative_imu_state_3d_stamped_constraint.h>
#include <bs_constraints/relative_pose/relative_pose_3d_stamped_with_extrinsics_constraint.h>
#include <bs_constraints/visual/euclidean_reprojection_constraint.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::GraphVisualization, fuse_core::SensorModel);

namespace bs_models {

using namespace graph_visualization;

GraphVisualization::GraphVisualization()
    : fuse_core::AsyncSensorModel(2),
      throttled_measurement_callback_(
          std::bind(&GraphVisualization::processMeasurements, this,
                    std::placeholders::_1)),
      throttled_image_callback_(std::bind(&GraphVisualization::processImage,
                                          this, std::placeholders::_1),
                                nullptr, ros::Duration(0.25)) {}

void GraphVisualization::onInit() {
  // Read settings from the parameter sever
  params_.loadFromROS(private_node_handle_);
  if (!params_.save_path.empty()) {
    if (!boost::filesystem::exists(params_.save_path)) {
      BEAM_ERROR("Invalid save path, path must exist: {}", params_.save_path);
      throw std::runtime_error{"invalid save path"};
    }
    std::string time_now =
        beam::ConvertTimeToDate(std::chrono::system_clock::now());
    save_path_ = beam::CombinePaths(params_.save_path, time_now);
    boost::filesystem::create_directory(save_path_);
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

  // todo: hardcoded topic, we need to reference raw topics globally
  image_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Image>(
      "/F1/image/", 10, &ThrottledImageCallback::callback,
      &throttled_image_callback_, ros::TransportHints().tcpNoDelay(false));

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
    Eigen::Matrix4d T_WORLD_BASELINK;
    const auto position = bs_common::GetPosition(graph_msg, timestamp);
    const auto orientation = bs_common::GetOrientation(graph_msg, timestamp);
    bs_common::FusePoseToEigenTransform(*position, *orientation,
                                        T_WORLD_BASELINK);

    // get landmarks in image
    auto lm_ids = landmark_container_->GetLandmarkIDsInImage(timestamp);
    for (const auto id : lm_ids) {
      Eigen::Vector2d pixel = landmark_container_->GetValue(timestamp, id);
      cv::Point m(pixel[0], pixel[1]);
      const auto lm_variable = bs_common::GetLandmark(graph_msg, id);
      if (lm_variable) {
        Eigen::Vector3d point =
            (T_CAM_BASELINK * beam::InvertTransform(T_WORLD_BASELINK) *
             lm_variable->point().homogeneous())
                .hnormalized();
        Eigen::Vector2d projected;
        if (cam_model_->ProjectPoint(point, projected)) {
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
  }
}

pcl::PointCloud<pcl::PointXYZRGBL>
    graph_visualization::GetGraphRelativeImuConstraintsAsCloud(
        const fuse_core::Graph& graph, double point_spacing,
        double frame_size) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  const auto constraints = graph.getConstraints();
  for (auto it = constraints.begin(); it != constraints.end(); it++) {
    if (it->type() != "bs_constraints::RelativeImuState3DStampedConstraint") {
      continue;
    }
    auto c = dynamic_cast<
        const bs_constraints::RelativeImuState3DStampedConstraint&>(*it);

    const auto& variable_uuids = c.variables();

    // get all pose variables
    std::vector<fuse_variables::Position3DStamped> positions;
    std::vector<fuse_variables::Orientation3DStamped> orientations;
    for (const auto& variable_uuid : variable_uuids) {
      if (!graph.variableExists(variable_uuid)) {
        BEAM_ERROR(
            "Invalid variable uuid query, uuid {} does not exist in graph",
            to_string(variable_uuid));
        throw std::runtime_error{"invalid variable query"};
      }
      const auto& var = graph.getVariable(variable_uuid);
      if (var.type() == "fuse_variables::Position3DStamped") {
        positions.push_back(
            dynamic_cast<const fuse_variables::Position3DStamped&>(
                graph.getVariable(variable_uuid)));
      } else if (var.type() == "fuse_variables::Orientation3DStamped") {
        orientations.push_back(
            dynamic_cast<const fuse_variables::Orientation3DStamped&>(
                graph.getVariable(variable_uuid)));
      }
    }

    if (positions.size() != 2 || orientations.size() != 2) {
      BEAM_ERROR("Invalid transaction, expecting 2 positions and 2 "
                 "orientations, received {} and {}, respectively.",
                 positions.size(), orientations.size());
      throw std::runtime_error{"invalid constraint"};
    }

    // sort poses by time
    fuse_variables::Position3DStamped p1;
    fuse_variables::Position3DStamped p2;
    fuse_variables::Orientation3DStamped o1;
    fuse_variables::Orientation3DStamped o2;
    if (positions.at(0).stamp() < positions.at(1).stamp()) {
      p1 = positions.at(0);
      p2 = positions.at(1);
    } else {
      p1 = positions.at(1);
      p2 = positions.at(0);
    }
    if (orientations.at(0).stamp() < orientations.at(1).stamp()) {
      o1 = orientations.at(0);
      o2 = orientations.at(1);
    } else {
      o1 = orientations.at(1);
      o2 = orientations.at(0);
    }

    // draw start variable pose
    Eigen::Matrix4d T_World_Baselink1;
    bs_common::FusePoseToEigenTransform(p1, o1, T_World_Baselink1);
    pcl::PointCloud<pcl::PointXYZRGBL> frame1 =
        beam::CreateFrameCol(p1.stamp(), point_spacing, frame_size);
    beam::MergeFrameToCloud(cloud, frame1, T_World_Baselink1);

    // draw end variable pose
    Eigen::Matrix4d T_World_Baselink2;
    bs_common::FusePoseToEigenTransform(p2, o2, T_World_Baselink2);
    pcl::PointCloud<pcl::PointXYZRGBL> frame2 =
        beam::CreateFrameCol(p2.stamp(), point_spacing, frame_size);
    beam::MergeFrameToCloud(cloud, frame2, T_World_Baselink2);

    // draw measured delta
    Eigen::Matrix4d T_Baselink1_Baselink2_Measured = c.getRelativePose();
    Eigen::Matrix4d T_World_Baselink2_Measured =
        T_World_Baselink1 * T_Baselink1_Baselink2_Measured;
    Eigen::Vector3d p_start(p1.x(), p1.y(), p1.z());
    Eigen::Vector3d p_end = T_World_Baselink2_Measured.block(0, 3, 3, 1);
    srand(p1.stamp().toNSec());
    uint8_t r = beam::randi(0, 255);
    uint8_t g = beam::randi(0, 255);
    uint8_t b = beam::randi(0, 255);
    pcl::PointCloud<pcl::PointXYZRGBL> line =
        bs_common::DrawLine(p_start, p_end, 0, r, g, b);
    cloud += line;
  }

  return cloud;
}

void graph_visualization::SaveImuBiases(
    const std::map<int64_t, bs_common::ImuBiases>& biases,
    const std::string& save_path, const std::string& filename) {
  std::string path = beam::CombinePaths(save_path, filename + ".csv");
  std::ofstream file;
  file.open(path);
  file << "#timestamp[Ns],AccelBiasX[m/s2],AccelBiasY[m/s2],AccelBiasZ[m/"
          "s2],GyroBiasX[raw/s],GyroBiasY[raw/s],GyroBiasZ[raw/s]";
  for (const auto& [t, bias] : biases) {
    file << t << "," << bias.a_x << "," << bias.a_y << "," << bias.a_z << ","
         << bias.g_x << "," << bias.g_y << "," << bias.g_z << "\n";
  }
  file.close();
}

pcl::PointCloud<pcl::PointXYZRGBL>
    graph_visualization::GetGraphGravityConstraintsAsCloud(
        const fuse_core::Graph& graph, double point_spacing, double frame_size,
        double g_length) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  const auto constraints = graph.getConstraints();
  for (auto it = constraints.begin(); it != constraints.end(); it++) {
    if (it->type() != "bs_constraints::GravityAlignmentStampedConstraint") {
      continue;
    }
    auto c =
        dynamic_cast<const bs_constraints::GravityAlignmentStampedConstraint&>(
            *it);

    const auto& variable_uuids = c.variables();
    if (variable_uuids.size() != 1) {
      BEAM_ERROR("Invalid number of variables returned for "
                 "GravityAlignmentStampedConstraint");
      throw std::runtime_error{"invalid constraint"};
    }
    const auto& variable_uuid = variable_uuids.at(0);
    if (!graph.variableExists(variable_uuid)) {
      BEAM_ERROR("Invalid variable uuid query, uuid {} does not exist in graph",
                 to_string(variable_uuid));
      throw std::runtime_error{"invalid variable query"};
    }
    const auto& o_var = graph.getVariable(variable_uuid);
    if (o_var.type() != "fuse_variables::Orientation3DStamped") {
      BEAM_ERROR("Invalid variable type returned for "
                 "GravityAlignmentStampedConstraint");
      throw std::runtime_error{"invalid constraint"};
    }

    fuse_variables::Orientation3DStamped o =
        dynamic_cast<const fuse_variables::Orientation3DStamped&>(
            graph.getVariable(variable_uuid));

    // find the pose variable in the graph
    auto position_uuid = fuse_core::uuid::generate(
        "fuse_variables::Position3DStamped", o.stamp(), fuse_core::uuid::NIL);
    if (!graph.variableExists(position_uuid)) {
      BEAM_ERROR("no position variable associated with orientation");
      throw std::runtime_error{"invalid variable query"};
    }
    const auto& p_var = graph.getVariable(position_uuid);
    fuse_variables::Position3DStamped p =
        dynamic_cast<const fuse_variables::Position3DStamped&>(
            graph.getVariable(position_uuid));

    // draw pose
    Eigen::Matrix4d T_World_Baselink;
    bs_common::FusePoseToEigenTransform(p, o, T_World_Baselink);
    pcl::PointCloud<pcl::PointXYZRGBL> frame =
        beam::CreateFrameCol(p.stamp(), point_spacing, frame_size);
    beam::MergeFrameToCloud(cloud, frame, T_World_Baselink);
    Eigen::Matrix3d R_World_Baselink_Measured =
        T_World_Baselink.block(0, 0, 3, 3);

    // get pose of constraint
    Eigen::Vector3d g_in_World_True{0, 0, -1};
    Eigen::Vector3d g_in_Baselink_Measured =
        g_length * c.gravity_in_baselink().normalized();
    Eigen::Vector3d g_in_World_Measured =
        R_World_Baselink_Measured * g_in_Baselink_Measured;
    Eigen::Vector3d p_start = T_World_Baselink.block(0, 3, 3, 1);

    pcl::PointCloud<pcl::PointXYZRGBL> line1 = bs_common::DrawLine(
        p_start, p_start + g_in_World_Measured, 0, 255, 100, 0);
    pcl::PointCloud<pcl::PointXYZRGBL> line2 = bs_common::DrawLine(
        p_start, p_start + g_length * g_in_World_True, 0, 255, 255, 0);
    cloud += line1;
    cloud += line2;
  }

  return cloud;
}

pcl::PointCloud<pcl::PointXYZRGBL>
    graph_visualization::GetGraphCameraLandmarksAsCloud(
        const fuse_core::Graph& graph) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  const auto constraints = graph.getConstraints();
  for (auto it = constraints.begin(); it != constraints.end(); it++) {
    if (it->type() != "bs_constraints::EuclideanReprojectionConstraint") {
      continue;
    }
    auto c =
        dynamic_cast<const bs_constraints::EuclideanReprojectionConstraint&>(
            *it);
    for (const auto& variable_uuid : c.variables()) {
      const auto& var = graph.getVariable(variable_uuid);
      if (var.type() != "fuse_variables::Point3DLandmark") { continue; }
      fuse_variables::Point3DLandmark P_World =
          dynamic_cast<const fuse_variables::Point3DLandmark&>(
              graph.getVariable(variable_uuid));
      pcl::PointXYZRGBL p;
      p.x = P_World.x();
      p.y = P_World.y();
      p.z = P_World.z();
      p.label = P_World.id();
      srand(P_World.id());
      p.r = beam::randi(0, 255);
      p.g = beam::randi(0, 255);
      p.b = beam::randi(0, 255);
      cloud.push_back(p);
    }
  }

  return cloud;
}

pcl::PointCloud<pcl::PointXYZRGBL>
    graph_visualization::GetGraphRelativePoseWithExtrinsicsConstraintsAsCloud(
        const fuse_core::Graph& graph, const std::string& source) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  const auto constraints = graph.getConstraints();
  for (auto it = constraints.begin(); it != constraints.end(); it++) {
    if (it->type() !=
        "bs_constraints::RelativePose3DStampedWithExtrinsicsConstraint") {
      continue;
    }
    auto c = dynamic_cast<
        const bs_constraints::RelativePose3DStampedWithExtrinsicsConstraint&>(
        *it);

    if (!source.empty()) {
      std::stringstream ss;
      c.print(ss);
      std::string str = ss.str();
      if (str.find(source) == std::string::npos) { continue; }
    }

    const auto& variable_uuids = c.variables();

    // get all pose variables
    std::vector<fuse_variables::Position3DStamped> positions;
    std::vector<fuse_variables::Orientation3DStamped> orientations;
    bs_variables::Position3D extrinsics_position;
    bs_variables::Orientation3D extrinsics_orientation;
    for (const auto& variable_uuid : variable_uuids) {
      if (!graph.variableExists(variable_uuid)) {
        BEAM_ERROR(
            "Invalid variable uuid query, uuid {} does not exist in graph",
            to_string(variable_uuid));
        throw std::runtime_error{"invalid variable query"};
      }
      const auto& var = graph.getVariable(variable_uuid);
      if (var.type() == "fuse_variables::Position3DStamped") {
        positions.push_back(
            dynamic_cast<const fuse_variables::Position3DStamped&>(
                graph.getVariable(variable_uuid)));
      } else if (var.type() == "fuse_variables::Orientation3DStamped") {
        orientations.push_back(
            dynamic_cast<const fuse_variables::Orientation3DStamped&>(
                graph.getVariable(variable_uuid)));
      } else if (var.type() == "bs_variables::Position3D") {
        extrinsics_position = dynamic_cast<const bs_variables::Position3D&>(
            graph.getVariable(variable_uuid));
      } else if (var.type() == "bs_variables::Orientation3D") {
        extrinsics_orientation =
            dynamic_cast<const bs_variables::Orientation3D&>(
                graph.getVariable(variable_uuid));
      } else {
        BEAM_WARN("Unknown variable type: {}", var.type());
      }
    }

    if (positions.size() != 2 || orientations.size() != 2) {
      BEAM_ERROR("Invalid transaction, expecting 2 positions and 2 "
                 "orientations, received {} and {}, respectively.",
                 positions.size(), orientations.size());
      throw std::runtime_error{"invalid constraint"};
    }

    // sort poses by time
    fuse_variables::Position3DStamped p1;
    fuse_variables::Position3DStamped p2;
    fuse_variables::Orientation3DStamped o1;
    fuse_variables::Orientation3DStamped o2;
    if (positions.at(0).stamp() < positions.at(1).stamp()) {
      p1 = positions.at(0);
      p2 = positions.at(1);
    } else {
      p1 = positions.at(1);
      p2 = positions.at(0);
    }
    if (orientations.at(0).stamp() < orientations.at(1).stamp()) {
      o1 = orientations.at(0);
      o2 = orientations.at(1);
    } else {
      o1 = orientations.at(1);
      o2 = orientations.at(0);
    }

    // draw start variable pose
    Eigen::Matrix4d T_World_Baselink1;
    bs_common::FusePoseToEigenTransform(p1, o1, T_World_Baselink1);
    pcl::PointCloud<pcl::PointXYZRGBL> frame1 =
        beam::CreateFrameCol(p1.stamp(), 0.01, 0.15);
    beam::MergeFrameToCloud(cloud, frame1, T_World_Baselink1);

    // draw end variable pose
    Eigen::Matrix4d T_World_Baselink2;
    bs_common::FusePoseToEigenTransform(p2, o2, T_World_Baselink2);
    pcl::PointCloud<pcl::PointXYZRGBL> frame2 =
        beam::CreateFrameCol(p2.stamp(), 0.01, 0.15);
    beam::MergeFrameToCloud(cloud, frame2, T_World_Baselink2);

    // get extrinsics
    Eigen::Matrix4d T_Baselink_Sensor = Eigen::Matrix4d::Identity();
    T_Baselink_Sensor(0, 3) = extrinsics_position.x();
    T_Baselink_Sensor(1, 3) = extrinsics_position.y();
    T_Baselink_Sensor(2, 3) = extrinsics_position.z();
    Eigen::Quaterniond q_tmp(
        extrinsics_orientation.w(), extrinsics_orientation.x(),
        extrinsics_orientation.y(), extrinsics_orientation.z());
    T_Baselink_Sensor.block(0, 0, 3, 3) = q_tmp.toRotationMatrix();

    // draw measured delta
    fuse_core::Vector7d delta = c.delta();
    Eigen::Quaterniond q(delta[3], delta[4], delta[5], delta[6]);
    Eigen::Matrix3d R(q);
    Eigen::Matrix4d T_Sensor1_Sensor2_Measured = Eigen::Matrix4d::Identity();
    T_Sensor1_Sensor2_Measured.block(0, 0, 3, 3) = R;
    T_Sensor1_Sensor2_Measured(0, 3) = delta[0];
    T_Sensor1_Sensor2_Measured(1, 3) = delta[1];
    T_Sensor1_Sensor2_Measured(2, 3) = delta[2];

    Eigen::Matrix4d T_World_Baselink2_Measured =
        T_World_Baselink1 * T_Baselink_Sensor * T_Sensor1_Sensor2_Measured *
        beam::InvertTransform(T_Baselink_Sensor);

    Eigen::Vector3d p_start(p1.x(), p1.y(), p1.z());
    Eigen::Vector3d p_end = T_World_Baselink2_Measured.block(0, 3, 3, 1);
    srand(p1.stamp().toNSec());
    uint8_t r = beam::randi(0, 255);
    uint8_t g = beam::randi(0, 255);
    uint8_t b = beam::randi(0, 255);
    double entropy =
        bs_common::ShannonEntropyFromPoseCovariance(c.covariance());
    pcl::PointCloud<pcl::PointXYZRGBL> line =
        bs_common::DrawLine(p_start, p_end, entropy, r, g, b);
    cloud += line;
  }

  return cloud;
}

void GraphVisualization::processImage(const sensor_msgs::Image::ConstPtr& msg) {
  image_buffer_[msg->header.stamp.toNSec()] = *msg;
}

void GraphVisualization::processMeasurements(
    const bs_common::CameraMeasurementMsg::ConstPtr& msg) {
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