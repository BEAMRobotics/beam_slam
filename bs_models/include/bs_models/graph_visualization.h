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

  /**
   * @brief Default Constructor
   */
  GraphVisualization();

  /**
   * @brief Default Destructor
   */
  ~GraphVisualization() override = default;

private:
  /**
   * @brief Perform any required initialization for the sensor model
   *
   * This could include things like reading from the parameter server or
   * subscribing to topics. The class's node handles will be properly
   * initialized before SensorModel::onInit() is called. Spinning of the
   * callback queue will not begin until after the call to SensorModel::onInit()
   * completes.
   */
  void onInit() override;

  /**
   * @brief Subscribe to the input topics to start sending transactions to the
   * optimizer
   */
  void onStart() override;

  /**
   * @brief Unsubscribe to the input topics
   */
  void onStop() override {}

  /**
   * @brief Read in graph updates
   */
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

  void LoadDataFromGraph(fuse_core::Graph::ConstSharedPtr graph_msg);

  void Publish();

  void Save();

  pcl::PointCloud<pcl::PointXYZRGBL>
      GetGraphRelativeImuConstraintsAsCloud(const fuse_core::Graph& graph);

  // loadable parameters
  bs_parameters::models::GraphVisualizationParams params_;

  // publishers
  ros::Publisher poses_publisher_;
  ros::Publisher relative_pose_constraints_publisher_;
  ros::Publisher relative_imu_constraints_publisher_;

  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
  std::string save_path_;

  // data storage
  int counter_{0};
  pcl::PointCloud<pcl::PointXYZRGBL> graph_poses_;
  pcl::PointCloud<pcl::PointXYZRGBL> relative_pose_constraints_;
  pcl::PointCloud<pcl::PointXYZRGBL> relative_imu_constraints_;
  std::map<int64_t, bs_common::ImuBiases> imu_biases_;
};

} // namespace bs_models
