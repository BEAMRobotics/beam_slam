#include <bs_models/graph_visualization/helpers.h>

#include <filesystem>

#include <beam_utils/log.h>

#include <bs_common/conversions.h>
#include <bs_common/utils.h>
#include <bs_common/visualization.h>
#include <bs_constraints/global/gravity_alignment_stamped_constraint.h>
#include <bs_constraints/inertial/relative_imu_state_3d_stamped_constraint.h>
#include <bs_constraints/relative_pose/relative_pose_3d_stamped_with_extrinsics_constraint.h>
#include <bs_constraints/visual/euclidean_reprojection_constraint.h>

namespace bs_models::graph_visualization {

pcl::PointCloud<pcl::PointXYZRGBL> GetGraphRelativeImuConstraintsAsCloud(
    const fuse_core::Graph& graph, double point_spacing, double frame_size) {
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

void SaveImuBiases(const std::map<int64_t, bs_common::ImuBiases>& biases,
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
    GetGraphGravityConstraintsAsCloud(const fuse_core::Graph& graph,
                                      double point_spacing, double frame_size,
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
    GetGraphCameraLandmarksAsCloud(const fuse_core::Graph& graph) {
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
    GetGraphRelativePoseWithExtrinsicsConstraintsAsCloud(
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

void ExportGraphVisualization(const std::string& output_directory,
                              const fuse_core::Graph& graph, double frame_size,
                              double point_spacing, double g_length,
                              int keypoints_circle_radius,
                              int keypoints_line_thickness) {
  if (!std::filesystem::exists(output_directory)) {
    BEAM_ERROR("cannot output graph, output direction does not exist: {}",
               output_directory);
    return;
  }

  // save graph
  std::ofstream graph_out(beam::CombinePaths(output_directory, "graph.txt"));
  graph.print(graph_out);
  graph_out.close();

  // save poses
  pcl::PointCloud<pcl::PointXYZRGBL> graph_poses =
      bs_common::GetGraphPosesAsCloud(graph);
  SaveCloud<pcl::PointXYZRGBL>(output_directory, "graph_poses.pcd",
                               graph_poses);

  // save lidar constraints
  pcl::PointCloud<pcl::PointXYZRGBL> lo_constraints =
      bs_common::GetGraphRelativePoseConstraintsAsCloud(graph,
                                                        "LidarOdometry::");
  if (lo_constraints.empty()) {
    lo_constraints = GetGraphRelativePoseWithExtrinsicsConstraintsAsCloud(
        graph, "LidarOdometry::");
  }
  if (!lo_constraints.empty()) {
    SaveCloud<pcl::PointXYZRGBL>(output_directory, "lo_constraints",
                                 lo_constraints);
  }

  // save imu constraints
  pcl::PointCloud<pcl::PointXYZRGBL> imu_constraints =
      GetGraphRelativeImuConstraintsAsCloud(graph, point_spacing, frame_size);
  SaveCloud<pcl::PointXYZRGBL>(output_directory, "imu_constraints",
                               imu_constraints);

  // save imu biases
  std::map<int64_t, bs_common::ImuBiases> biases_in_graph =
      bs_common::GetImuBiasesFromGraph(graph);
  if (!biases_in_graph.empty()) {
    SaveImuBiases(biases_in_graph, output_directory, "imu_biases");
  }

  // save gravity constraints
  pcl::PointCloud<pcl::PointXYZRGBL> gravity_constraints =
      GetGraphGravityConstraintsAsCloud(graph, point_spacing, frame_size,
                                        g_length);
  SaveCloud<pcl::PointXYZRGBL>(output_directory, "gravity_constraints",
                               gravity_constraints);
}

} // namespace bs_models::graph_visualization