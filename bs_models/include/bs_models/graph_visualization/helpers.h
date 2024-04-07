#pragma once

#include <fuse_core/graph.h>

#include <beam_utils/filesystem.h>
#include <beam_utils/pointclouds.h>

#include <bs_common/graph_access.h>

namespace bs_models::graph_visualization {

void ExportGraphVisualization(const std::string& output_directory,
                              const fuse_core::Graph& graph,
                              double frame_size = 0.15,
                              double point_spacing = 0.01,
                              double g_length = 0.25,
                              int keypoints_circle_radius = 2,
                              int keypoints_line_thickness = 3);

template <typename PointT>
void SaveCloud(const std::string& save_path, const std::string& cloud_name,
               const pcl::PointCloud<PointT>& cloud) {
  if (save_path.empty()) { return; }
  std::string filename = beam::CombinePaths(save_path, cloud_name + ".pcd");
  beam::SavePointCloud<PointT>(filename, cloud);
}

void SaveImuBiases(const std::map<int64_t, bs_common::ImuBiases>& biases,
                   const std::string& save_path, const std::string& filename);

pcl::PointCloud<pcl::PointXYZRGBL> GetGraphRelativeImuConstraintsAsCloud(
    const fuse_core::Graph& graph, double point_spacing, double frame_size);

pcl::PointCloud<pcl::PointXYZRGBL>
    GetGraphGravityConstraintsAsCloud(const fuse_core::Graph& graph,
                                      double point_spacing, double frame_size,
                                      double g_length);

pcl::PointCloud<pcl::PointXYZRGBL>
    GetGraphCameraLandmarksAsCloud(const fuse_core::Graph& graph);

pcl::PointCloud<pcl::PointXYZRGBL>
    GetGraphRelativePoseWithExtrinsicsConstraintsAsCloud(
        const fuse_core::Graph& graph, const std::string& source = "",
        bool draw_poses = true);

} // namespace bs_models::graph_visualization
