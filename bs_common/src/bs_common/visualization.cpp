#include <bs_common/visualization.h>

#include <beam_utils/pointclouds.h>
#include <fuse_constraints/relative_pose_3d_stamped_constraint.h>

#include <bs_common/matplotlibcpp.h>
#include <bs_common/utils.h>

namespace bs_common {

namespace plt = matplotlibcpp;

void PlotData(const std::vector<double>& x, const std::vector<double>& y,
              const std::string& save_path) {
  plt::plot(x, y);
  plt::save(save_path);
}

void PlotImuBiasesFromGraph(const fuse_core::Graph& graph,
                            const std::string& filepath) {
  const auto var_range = graph.getVariables();
  std::vector<double> t; // in s
  std::vector<double> gx;
  std::vector<double> gy;
  std::vector<double> gz;
  std::vector<double> ax;
  std::vector<double> ay;
  std::vector<double> az;
  for (auto it = var_range.begin(); it != var_range.end(); it++) {
    if (it->type() == "bs_variables::GyroscopeBias3DStamped") {
      auto v = dynamic_cast<const bs_variables::GyroscopeBias3DStamped&>(*it);
      gx.push_back(v.x());
      gy.push_back(v.y());
      gz.push_back(v.z());
      if (!t.empty()) {
        t.push_back(v.stamp().toSec() - t.at(0));
      } else {
        t.push_back(v.stamp().toSec());
      }

    } else if (it->type() == "bs_variables::AccelerationBias3DStamped") {
      auto v =
          dynamic_cast<const bs_variables::AccelerationBias3DStamped&>(*it);
      ax.push_back(v.x());
      ay.push_back(v.y());
      az.push_back(v.z());
    }
  }
  if (t.empty()) {
    BEAM_ERROR("no bias terms in graph, not plotting IMU biases");
    return;
  }

  plt::suptitle("IMU Biases");
  plt::subplot(2, 1, 1);
  plt::named_plot("gx", t, gx, "r");
  plt::named_plot("gy", t, gy, "g");
  plt::named_plot("gz", t, gz, "b");
  plt::legend();
  plt::subplot(2, 1, 2);
  plt::named_plot("ax", t, ax, "r");
  plt::named_plot("ay", t, ay, "g");
  plt::named_plot("az", t, az, "b");
  plt::legend();
  plt::xlabel("time elapsed (s)");
  plt::save(filepath);
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
} // namespace bs_common
