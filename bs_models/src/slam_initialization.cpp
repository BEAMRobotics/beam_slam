#include <bs_models/slam_initialization.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::SLAMInitialization, fuse_core::SensorModel)

namespace bs_models {

using namespace vision;

SLAMInitialization::SLAMInitialization()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_image_callback_(std::bind(&SLAMInitialization::processImage,
                                          this, std::placeholders::_1)) {}

void SLAMInitialization::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  slam_initialization_params_.loadFromROS(private_node_handle_);
}

void SLAMInitialization::onStart() {
  // subscribe to topics
  visual_measurement_subscriber_ =
      private_node_handle_.subscribe<sensor_msgs::Image>(
          ros::names::resolve(
              slam_initialization_params__.visual_measurement_topic),
          1000, &ThrottledMeasuremenCallback::callback,
          &throttled_measurement_callback_,
          ros::TransportHints().tcpNoDelay(false));

  imu_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Image>(
      ros::names::resolve(slam_initialization_params__.imu_topic), 1000,
      &ThrottledIMUCallback::callback, &throttled_imu_callback_,
      ros::TransportHints().tcpNoDelay(false));

  lidar_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Image>(
      ros::names::resolve(slam_initialization_params__.lidar_topic), 1000,
      &ThrottledLidarCallback::callback, &throttled_lidar_callback_,
      ros::TransportHints().tcpNoDelay(false));
}

void SLAMInitialization::processMeasurements(
    const CameraMeasurementMsg::ConstPtr& msg) {}

void SLAMInitialization::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {}

void SLAMInitialization::processLidar(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {}

} // namespace bs_models
