#pragma once

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/throttled_callback.h>

#include <opencv2/core.hpp>

#include <beam_calibration/CameraModel.h>
#include <beam_containers/LandmarkContainer.h>
#include <beam_utils/pointclouds.h>

#include <bs_common/bs_msgs.h>
#include <bs_common/extrinsics_lookup_online.h>
#include <bs_parameters/models/graph_visualization_params.h>

namespace bs_models {

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
  using ConstraintTypeMap =
      std::unordered_map<fuse_core::UUID, std::string, fuse_core::uuid::hash>;

  // for each position, store map from uuid to constraint type for priors
  struct VariableConnectivity {
    ros::Time stamp;
    ConstraintTypeMap absolute_constraints;
    ConstraintTypeMap previous_constraints; // constraints to prev position
    ConstraintTypeMap next_constraints;     // constraints to next position
    VariableConnectivity(const ros::Time& time) : stamp(time) {}
  };

  using VariableConnectivityType =
      std::unordered_map<fuse_core::UUID, VariableConnectivity,
                         fuse_core::uuid::hash>;

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

  void ValidateGraphPriors(fuse_core::Graph::ConstSharedPtr graph_msg);

  void ValidateGraphConnectivity(fuse_core::Graph::ConstSharedPtr graph_msg);

  template <typename PointT>
  void PublishCloud(PublisherWithCounter& publisher,
                    const pcl::PointCloud<PointT>& cloud) {
    if (!params_.publish) { return; }
    sensor_msgs::PointCloud2 ros_cloud = beam::PCLToROS<PointT>(
        cloud, current_time_, extrinsics_.GetWorldFrameId(), publisher.counter);
    publisher.publisher.publish(ros_cloud);
    publisher.counter++;
  }

  void
      processMeasurements(const bs_common::CameraMeasurementMsg::ConstPtr& msg);

  bool HasImuConstraint(const ConstraintTypeMap& constraints) const;

  fuse_core::UUID
      GetLastVariable(const VariableConnectivityType& connectivity) const;

  fuse_core::UUID
      GetFirstVariable(const VariableConnectivityType& connectivity) const;

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

  /// @brief vo visualization things
  ros::Subscriber feature_track_subscriber_;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_containers::LandmarkContainer> landmark_container_;
  std::map<uint64_t, sensor_msgs::Image> image_buffer_;
  ros::Publisher image_publisher_;

  using ThrottledMeasurementCallback =
      fuse_core::ThrottledMessageCallback<bs_common::CameraMeasurementMsg>;
  ThrottledMeasurementCallback throttled_measurement_callback_;

  using ThrottledImageCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::Image>;
  ThrottledImageCallback throttled_image_callback_;

  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
  std::string save_path_;
  ros::Time current_time_;

  // graph validation
  std::unordered_set<fuse_core::UUID, fuse_core::uuid::hash>
      absolute_imu_constraints_;
  std::unordered_set<fuse_core::UUID, fuse_core::uuid::hash>
      absolute_pose_constraints_;
  std::unordered_map<std::string, fuse_core::UUID>
      extrinsics_constraints_; // T_child_parent -> uuid
  bool prior_found_on_first_{false};

  // parameters only tunable here
  double frame_size_{0.15};
  double point_spacing_{0.01};
  double g_length_{0.25};
  int keypoints_circle_radius_{2};
  int keypoints_line_thickness_{3};
};

} // namespace bs_models
