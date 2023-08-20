#include <bs_models/graph_visualization.h>

#include <boost/filesystem.hpp>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>

#include <beam_utils/filesystem.h>
#include <beam_utils/time.h>

#include <bs_common/conversions.h>
#include <bs_common/utils.h>
#include <bs_common/visualization.h>
#include <bs_constraints/inertial/relative_imu_state_3d_stamped_constraint.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::GraphVisualization, fuse_core::SensorModel);

namespace bs_models {

GraphVisualization::GraphVisualization() : fuse_core::AsyncSensorModel(2) {}

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
}

void GraphVisualization::onStart() {
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
}

void GraphVisualization::VisualizePoses(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud =
      bs_common::GetGraphPosesAsCloud(*graph_msg);
  PublishCloud<pcl::PointXYZRGBL>(poses_publisher_, cloud);
  SaveCloud<pcl::PointXYZRGBL>("graph_poses", cloud);
}

void GraphVisualization::VisualizeLidarRelativePoseConstraints(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud =
      bs_common::GetGraphRelativePoseConstraintsAsCloud(*graph_msg,
                                                        "LidarOdometry::");
  PublishCloud<pcl::PointXYZRGBL>(lidar_relative_pose_constraints_publisher_,
                                  cloud);
  SaveCloud<pcl::PointXYZRGBL>("relative_pose_constraints", cloud);
}

void GraphVisualization::VisualizeImuRelativeConstraints(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud =
      GetGraphRelativeImuConstraintsAsCloud(*graph_msg);
  PublishCloud<pcl::PointXYZRGBL>(relative_imu_constraints_publisher_, cloud);
  SaveCloud<pcl::PointXYZRGBL>("relative_imu_constraints", cloud);
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
  ax.data = biases_recent.a_z;
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
  SaveImuBiases(biases_in_graph,
                std::to_string(current_time_.toSec()) + "_imu_biases");
}

void GraphVisualization::VisualizeImuGravityConstraints(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  // TODO
  ROS_WARN_STREAM_THROTTLE(5,
                           "Need to implement: VisualizeImuGravityConstraints");
}

pcl::PointCloud<pcl::PointXYZRGBL>
    GraphVisualization::GetGraphRelativeImuConstraintsAsCloud(
        const fuse_core::Graph& graph) const {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  const auto constraints = graph.getConstraints();
  for (auto it = constraints.begin(); it != constraints.end(); it++) {
    if (it->type() !=
        "bs_constraints::inertial::RelativeImuState3DStampedConstraint") {
      continue;
    }
    auto c = dynamic_cast<
        const bs_constraints::inertial::RelativeImuState3DStampedConstraint&>(
        *it);

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
        beam::CreateFrameCol(p1.stamp(), 0.01, 0.15);
    beam::MergeFrameToCloud(cloud, frame1, T_World_Baselink1);

    // draw end variable pose
    Eigen::Matrix4d T_World_Baselink2;
    bs_common::FusePoseToEigenTransform(p2, o2, T_World_Baselink2);
    pcl::PointCloud<pcl::PointXYZRGBL> frame2 =
        beam::CreateFrameCol(p2.stamp(), 0.01, 0.15);
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
        DrawLine(p_start, p_end, 0, r, g, b);
    cloud += line;
  }

  return cloud;
}

void GraphVisualization::SaveImuBiases(
    const std::map<int64_t, bs_common::ImuBiases>& biases,
    const std::string& filename) const {
  // TODO
  ROS_WARN_STREAM_THROTTLE(5, "Need to implement SaveImuBiases");
}

} // namespace bs_models