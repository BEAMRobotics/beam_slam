#pragma once

#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/variable.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <bs_common/graph_access.h>
#include <bs_common/imu_state.h>

namespace bs_common {

void PlotData(const std::vector<double>& x, const std::vector<double>& y,
              const std::string& save_path);

void PlotImuBiases(const std::map<int64_t, ImuBiases>& biases,
                   const std::string& filepath);

void PlotImuBiasesFromGraph(const fuse_core::Graph& graph,
                            const std::string& filepath);

// Draw a coordinate frame with a velocity vector.
// For the frame: RGB corresponds to the frames (x-r, y-g, z-b) and label
// corresponds to the timestamp For the velocity vector: RGB is always magenta,
// label corresponds to the magnitude of velocity in mm/s (m/s * 1000)
pcl::PointCloud<pcl::PointXYZRGBL>
    ImuStateToCloudInWorld(const ImuState& imu_state);

// Get pointcloud with position, orientations and velocities saves as coordinate
// frames
pcl::PointCloud<pcl::PointXYZRGBL>
    GetGraphPosesAsCloud(const fuse_core::Graph& graph);

pcl::PointCloud<pcl::PointXYZRGBL>
    TrajectoryToCloud(const std::map<uint64_t, Eigen::Matrix4d>& trajectory);

pcl::PointCloud<pcl::PointXYZRGBL>
    GetGraphRelativePoseConstraintsAsCloud(const fuse_core::Graph& graph);

pcl::PointCloud<pcl::PointXYZRGBL> DrawLine(const Eigen::Vector3d& p1,
                                            const Eigen::Vector3d& p2,
                                            float label = 0, uint8_t r = 255,
                                            uint8_t g = 0, uint8_t b = 0,
                                            double increment = 0.01);
} // namespace bs_common
