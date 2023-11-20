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

namespace bs_models {

/**
 * @brief this is a lightweight version of the GraphVisualization which only
 * displays visual keypoints and poses
 */
class GraphPublisher : public fuse_core::AsyncSensorModel {
public:
  FUSE_SMART_PTR_DEFINITIONS(GraphPublisher);

  struct PublisherWithCounter {
    ros::Publisher publisher;
    int counter{0};
  };

  /**
   * @brief Default Constructor
   */
  GraphPublisher();

  /**
   * @brief Default Destructor
   */
  ~GraphPublisher() override = default;

private:
  void onInit() override;
  void onStart() override;
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;
  void PublishPoses(fuse_core::Graph::ConstSharedPtr graph_msg);
  void PublishCameraLandmarks(fuse_core::Graph::ConstSharedPtr graph_msg);

  template <typename PointT>
  void PublishCloud(PublisherWithCounter& publisher,
                    const pcl::PointCloud<PointT>& cloud) {
    sensor_msgs::PointCloud2 ros_cloud = beam::PCLToROS<PointT>(
        cloud, current_time_, extrinsics_.GetWorldFrameId(), publisher.counter);
    publisher.publisher.publish(ros_cloud);
    publisher.counter++;
  }

  void processCameraMeasurements(
      const bs_common::CameraMeasurementMsg::ConstPtr& msg);

  // publishers
  PublisherWithCounter graph_path_publisher_;
  PublisherWithCounter graph_odom_publisher_; // these are marginalized poses
  PublisherWithCounter camera_landmarks_publisher_;
  PublisherWithCounter image_publisher_;

  /// @brief vo visualization things
  ros::Subscriber feature_track_subscriber_;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_containers::LandmarkContainer> landmark_container_;
  std::map<uint64_t, sensor_msgs::Image> image_buffer_;

  using ThrottledMeasurementCallback =
      fuse_core::ThrottledMessageCallback<bs_common::CameraMeasurementMsg>;
  ThrottledMeasurementCallback throttled_measurement_callback_;

  using ThrottledImageCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::Image>;
  ThrottledImageCallback throttled_image_callback_;

  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
  ros::Time current_time_;
  std::map<ros::Time, Eigen::Matrix4d> poses_last_;

  // parameters only tunable here
  double frame_size_{0.15};
  double point_spacing_{0.01};
  int keypoints_circle_radius_{2};
  int keypoints_line_thickness_{3};
};

} // namespace bs_models
