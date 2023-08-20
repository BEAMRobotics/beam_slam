#include <bs_common/visualization.h>

#include <beam_utils/math.h>
#include <beam_utils/pointclouds.h>
#include <fuse_constraints/relative_pose_3d_stamped_constraint.h>

#include <bs_common/conversions.h>
#include <bs_common/graph_access.h>
#include <bs_common/matplotlibcpp.h>
#include <bs_common/utils.h>

namespace plt = matplotlibcpp;

namespace bs_common {

void PlotData(const std::vector<double>& x, const std::vector<double>& y,
              const std::string& save_path) {
  plt::plot(x, y);
  plt::save(save_path);
}

void PlotImuBiases(const std::map<int64_t, ImuBiases>& biases,
                   const std::string& filepath) {
  if (biases.empty()) {
    BEAM_ERROR("no bias terms in graph, not plotting IMU biases");
    return;
  }

  std::vector<double> ts; // in s
  std::vector<double> gx;
  std::vector<double> gy;
  std::vector<double> gz;
  std::vector<double> ax;
  std::vector<double> ay;
  std::vector<double> az;
  for (const auto& [t, b] : biases) {
    ts.push_back(t * 1e-9);
    gx.push_back(b.g_x);
    gy.push_back(b.g_y);
    gz.push_back(b.g_z);
    ax.push_back(b.a_x);
    ay.push_back(b.a_y);
    az.push_back(b.a_z);
  }

  plt::suptitle("IMU Biases");
  plt::subplot(2, 1, 1);
  plt::named_plot("gx", ts, gx, "r");
  plt::named_plot("gy", ts, gy, "g");
  plt::named_plot("gz", ts, gz, "b");
  plt::legend();
  plt::subplot(2, 1, 2);
  plt::named_plot("ax", ts, ax, "r");
  plt::named_plot("ay", ts, ay, "g");
  plt::named_plot("az", ts, az, "b");
  plt::legend();
  plt::xlabel("time elapsed (s)");
  plt::save(filepath);
}

void PlotImuBiasesFromGraph(const fuse_core::Graph& graph,
                            const std::string& filepath) {
  std::map<int64_t, ImuBiases> biases = GetImuBiasesFromGraph(graph);
  PlotImuBiases(biases, filepath);
}

pcl::PointCloud<pcl::PointXYZRGBL>
    ImuStateToCloudInWorld(const ImuState& imu_state) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud_in_world;
  Eigen::Matrix3d R_World_Imu = imu_state.OrientationMat();
  Eigen::Matrix4d T_World_Imu;
  T_World_Imu.block(0, 0, 3, 3) = R_World_Imu;
  T_World_Imu.block(0, 3, 3, 1) = imu_state.PositionVec();

  Eigen::Vector3d v_in_imu = imu_state.VelocityVec();
  Eigen::Vector3d v_in_world = R_World_Imu * v_in_imu;
  Eigen::Vector3d v_dir_in_world = v_in_world.normalized();
  double v_mag_in_world = v_in_imu.norm();

  double increment = 0.01;
  double length = 0.3;
  pcl::PointCloud<pcl::PointXYZRGBL> frame =
      beam::CreateFrameCol(imu_state.Stamp(), increment, length);
  beam::MergeFrameToCloud(cloud_in_world, frame, T_World_Imu);

  double cur_length{0};
  while (cur_length < length) {
    Eigen::Vector3d p = cur_length * v_dir_in_world + imu_state.PositionVec();
    pcl::PointXYZRGBL p_labeled;
    p_labeled.x = p[0];
    p_labeled.y = p[1];
    p_labeled.z = p[2];
    p_labeled.r = 255;
    p_labeled.g = 0;
    p_labeled.b = 255;
    p_labeled.label = v_mag_in_world * 1000;
    cloud_in_world.points.emplace_back(p_labeled);
    cur_length += increment;
  }
  return cloud_in_world;
}

pcl::PointCloud<pcl::PointXYZRGBL>
    TrajectoryToCloud(const std::map<uint64_t, Eigen::Matrix4d>& trajectory) {
  double increment = 0.01;
  double length = 0.3;
  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  for (const auto& [stamp, T] : trajectory) {
    ros::Time stamp_ros;
    stamp_ros.fromNSec(stamp);
    pcl::PointCloud<pcl::PointXYZRGBL> frame =
        beam::CreateFrameCol(stamp_ros, increment, length);
    beam::MergeFrameToCloud(cloud, frame, T);
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZRGBL>
    GetGraphPosesAsCloud(const fuse_core::Graph& graph) {
  // save as IMU state so we can reuse the pointcloud function above
  std::map<uint64_t, ImuState> poses;
  const auto var_range = graph.getVariables();
  for (auto it = var_range.begin(); it != var_range.end(); it++) {
    if (it->type() == "fuse_variables::Position3DStamped") {
      auto v = dynamic_cast<const fuse_variables::Position3DStamped&>(*it);
      uint64_t t = v.stamp().toNSec();
      if (poses.find(t) != poses.end()) {
        poses.at(t).SetPosition(v.x(), v.y(), v.z());
      } else {
        ImuState state(v.stamp());
        state.SetPosition(v.x(), v.y(), v.z());
        poses.emplace(t, state);
      }
    } else if (it->type() == "fuse_variables::Orientation3DStamped") {
      auto v = dynamic_cast<const fuse_variables::Orientation3DStamped&>(*it);
      uint64_t t = v.stamp().toNSec();
      if (poses.find(t) != poses.end()) {
        poses.at(t).SetOrientation(v.w(), v.x(), v.y(), v.z());
      } else {
        ImuState state(v.stamp());
        state.SetOrientation(v.w(), v.x(), v.y(), v.z());
        poses.emplace(t, state);
      }
    } else if (it->type() == "fuse_variables::VelocityLinear3DStamped") {
      auto v =
          dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(*it);
      uint64_t t = v.stamp().toNSec();
      if (poses.find(t) != poses.end()) {
        poses.at(t).SetVelocity(v.x(), v.y(), v.z());
      } else {
        ImuState state(v.stamp());
        state.SetVelocity(v.x(), v.y(), v.z());
        poses.emplace(t, state);
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  for (const auto& [t, imu_state] : poses) {
    auto pose_cloud = ImuStateToCloudInWorld(imu_state);
    cloud += pose_cloud;
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZRGBL>
    GetGraphRelativePoseConstraintsAsCloud(const fuse_core::Graph& graph,
                                           const std::string& source) {
  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  const auto constraints = graph.getConstraints();
  for (auto it = constraints.begin(); it != constraints.end(); it++) {
    if (it->type() != "fuse_constraints::RelativePose3DStampedConstraint") {
      continue;
    }
    auto c =
        dynamic_cast<const fuse_constraints::RelativePose3DStampedConstraint&>(
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

    // draw measured delta
    fuse_core::Vector7d delta = c.delta();
    Eigen::Quaterniond q(delta[3], delta[4], delta[5], delta[6]);
    Eigen::Matrix3d R_Baselink1_Baselink2(q);
    Eigen::Matrix4d T_Baselink1_Baselink2_Measured =
        Eigen::Matrix4d::Identity();
    T_Baselink1_Baselink2_Measured.block(0, 0, 3, 3) = R_Baselink1_Baselink2;
    T_Baselink1_Baselink2_Measured(0, 3) = delta[0];
    T_Baselink1_Baselink2_Measured(1, 3) = delta[1];
    T_Baselink1_Baselink2_Measured(2, 3) = delta[2];
    Eigen::Matrix4d T_World_Baselink2_Measured =
        T_World_Baselink1 * T_Baselink1_Baselink2_Measured;
    Eigen::Vector3d p_start(p1.x(), p1.y(), p1.z());
    Eigen::Vector3d p_end = T_World_Baselink2_Measured.block(0, 3, 3, 1);
    srand(p1.stamp().toNSec());
    uint8_t r = beam::randi(0, 255);
    uint8_t g = beam::randi(0, 255);
    uint8_t b = beam::randi(0, 255);
    double entropy =
        bs_common::ShannonEntropyFromPoseCovariance(c.covariance());
    pcl::PointCloud<pcl::PointXYZRGBL> line =
        DrawLine(p_start, p_end, entropy, r, g, b);
    cloud += line;
  }

  return cloud;
}

pcl::PointCloud<pcl::PointXYZRGBL> DrawLine(const Eigen::Vector3d& p1,
                                            const Eigen::Vector3d& p2,
                                            float label, uint8_t r, uint8_t g,
                                            uint8_t b, double increment) {
  auto diff = p2 - p1;
  double len = diff.norm();
  auto dir = diff.normalized();
  double cur_length = 0;
  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  while (cur_length < len) {
    Eigen::Vector3d p_eig = p1 + cur_length * dir;
    pcl::PointXYZRGBL p;
    p.x = p_eig[0];
    p.y = p_eig[1];
    p.z = p_eig[2];
    p.r = r;
    p.g = g;
    p.b = b;
    p.label = label;
    cloud.push_back(p);
    cur_length += increment;
  }
  return cloud;
}

} // namespace bs_common
