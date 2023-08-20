#pragma once

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/fuse_macros.h>

#include <beam_utils/pointclouds.h>

#include <bs_common/extrinsics_lookup_online.h>
#include <bs_common/graph_access.h>
#include <bs_parameters/models/graph_visualization_params.h>

namespace bs_models {

using namespace bs_common;

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
  void onStop() override;
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  template <typename PointT>
  void PublishCloud(const PublisherWithCounter& publisher,
                    const pcl::PointCloud<PointT>& cloud) {
    if (!params_.publish) { return; }
    sensor_msgs::PointCloud2 ros_cloud = beam::PCLToROS<PointT>(
        cloud, current_time_, extrinsics_.GetWorldFrameId(), publisher.counter);
    poses_publisher_.publish(ros_cloud);
    publisher.counter++;
  }

  template <typename PointT>
  void SaveCloud(const std::string& cloud_name,
                 const pcl::PointCloud<PointT>& cloud) {
    if (save_path_.empty()) { return; }
    std::string stamp_str = std::to_string(current_time_.toSec());
    std::string filename =
        beam::CombinePaths(save_path_, stamp_str + "_" + cloud_name + ".pcd");
    beam::SavePointCloud<PointT>(filename, cloud);
  }

  void SaveImuBiases(const std::map<int64_t, bs_common::ImuBiases>& biases,
                     const std::string& filename) const;

  pcl::PointCloud<pcl::PointXYZRGBL> GetGraphRelativeImuConstraintsAsCloud(
      const fuse_core::Graph& graph) const;

  // loadable parameters
  bs_parameters::models::GraphVisualizationParams params_;

  // publishers
  PublisherWithCounter poses_publisher_;
  PublisherWithCounter relative_pose_constraints_publisher_;
  PublisherWithCounter relative_imu_constraints_publisher_;
  PublisherWithCounter gravity_constraints_publisher_;
  ros::Publisher gyro_biases_publisher_;
  ros::Publisher accel_biases_publisher_;

  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
  std::string save_path_;
  ros::Time current_time_;

  // data storage
  std::map<int64_t, bs_common::ImuBiases> imu_biases_all_;
};

} // namespace bs_models