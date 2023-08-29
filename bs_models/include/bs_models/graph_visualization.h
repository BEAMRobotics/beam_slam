#pragma once

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/fuse_macros.h>

#include <beam_utils/pointclouds.h>

#include <bs_common/extrinsics_lookup_online.h>
#include <bs_common/graph_access.h>
#include <bs_parameters/models/graph_visualization_params.h>

namespace bs_models {

using namespace bs_common;

namespace graph_visualization {

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
        const fuse_core::Graph& graph, const std::string& source);

} // namespace graph_visualization

class GraphVisualization : public fuse_core::AsyncSensorModel {
public:
  FUSE_SMART_PTR_DEFINITIONS(GraphVisualization);

  struct PublisherWithCounter {
    ros::Publisher publisher;
    int counter{0};
  };

  /**
   * @brief Default Constructor
   */
  GraphVisualization();

  /**
   * @brief Default Destructor
   */
  ~GraphVisualization() override = default;

private:
  void onInit() override;
  void onStart() override;
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  void VisualizePoses(fuse_core::Graph::ConstSharedPtr graph_msg);

  void VisualizeLidarRelativePoseConstraints(
      fuse_core::Graph::ConstSharedPtr graph_msg);

  void VisualizeImuRelativeConstraints(
      fuse_core::Graph::ConstSharedPtr graph_msg);

  void VisualizeImuBiases(fuse_core::Graph::ConstSharedPtr graph_msg);

  void VisualizeImuGravityConstraints(
      fuse_core::Graph::ConstSharedPtr graph_msg);

  void VisualizeCameraLandmarks(fuse_core::Graph::ConstSharedPtr graph_msg);

  template <typename PointT>
  void PublishCloud(PublisherWithCounter& publisher,
                    const pcl::PointCloud<PointT>& cloud) {
    if (!params_.publish) { return; }
    sensor_msgs::PointCloud2 ros_cloud = beam::PCLToROS<PointT>(
        cloud, current_time_, extrinsics_.GetWorldFrameId(), publisher.counter);
    publisher.publisher.publish(ros_cloud);
    publisher.counter++;
  }

  // loadable parameters
  bs_parameters::models::GraphVisualizationParams params_;

  // publishers
  PublisherWithCounter poses_publisher_;
  PublisherWithCounter lidar_relative_pose_constraints_publisher_;
  PublisherWithCounter relative_imu_constraints_publisher_;
  PublisherWithCounter gravity_constraints_publisher_;
  PublisherWithCounter camera_landmarks_publisher_;
  ros::Publisher imu_biases_publisher_gx_;
  ros::Publisher imu_biases_publisher_gy_;
  ros::Publisher imu_biases_publisher_gz_;
  ros::Publisher imu_biases_publisher_ax_;
  ros::Publisher imu_biases_publisher_ay_;
  ros::Publisher imu_biases_publisher_az_;

  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
  std::string save_path_;
  ros::Time current_time_;

  // parameters only tunable here
  double frame_size_{0.15};
  double point_spacing_{0.01};
  double g_length_{0.25};
};

} // namespace bs_models
