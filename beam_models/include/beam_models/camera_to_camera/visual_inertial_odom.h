#pragma once
// std
#include <queue>
// ros
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
// beam_slam
#include <beam_constraints/camera_to_camera/visual_constraint.h>
#include <beam_models/camera_to_camera/vio_initializer.h>
#include <beam_parameters/models/camera_params.h>
#include <beam_variables/position_3d.h>
// fuse
#include <fuse_core/async_sensor_model.h>
// libbeam
#include <beam_calibration/CameraModel.h>
#include <beam_cv/tracker/Tracker.h>

namespace beam_models { namespace camera_to_camera {

class VisualInertialOdom : public fuse_core::AsyncSensorModel {
public:
  SMART_PTR_DEFINITIONS(VisualInertialOdom);

  VisualInertialOdom();

  ~VisualInertialOdom() override = default;

  /**
   * @brief Callback for image processing, this callback will add visual
   * constraints and triangulate new landmarks when required
   * @param[in] msg - The image to process
   */
  void processImage(const sensor_msgs::Image::ConstPtr& msg);

  /**
   * @brief Callback for image processing, this callback will add visual
   * constraints and triangulate new landmarks when required
   * @param[in] msg - The image to process
   */
  void processIMU(const sensor_msgs::Imu::ConstPtr& msg);

protected:
  fuse_core::UUID device_id_; //!< The UUID of this device

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
   * @brief Subscribe to the input topic to start sending transactions to the
   * optimizer
   */
  void onStart() override {}

  /**
   * @brief Unsubscribe to the input topic
   */
  void onStop() override;

  /**
   * @brief Callback for when a newly optimized graph is available
   */
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

private:
  /**
   * @brief Creates initial variables for the map
   */
  std::shared_ptr<fuse_core::Transaction> InitializeMap(ros::Time cur_time);

  /**
   * @brief Converts ros image message to opencv image
   */
  cv::Mat ExtractImage(const sensor_msgs::Image& msg);

  /**
   * @brief Helper function to add constraints for already existing landmarks
   * @param track feature track of current image
   */
  fuse_variables::Orientation3DStamped::SharedPtr
      GetCameraOrientation(const ros::Time& stamp);

  /**
   * @brief Helper function to add constraints for already existing landmarks
   * @param track feature track of current image
   */
  fuse_variables::Position3DStamped::SharedPtr
      GetCameraPosition(const ros::Time& stamp);

  /**
   * @brief Helper function to add constraints for already existing landmarks
   * @param track feature track of current image
   */
  fuse_variables::Position3D::SharedPtr GetLandmark(uint64_t landmark_id);

protected:
  // loadable camera parameters
  beam_parameters::models::CameraParams params_;
  // topic subscribers and buffers
  ros::Subscriber image_subscriber_;
  ros::Subscriber imu_subscriber_;
  std::queue<sensor_msgs::Image> image_buffer_;
  std::queue<sensor_msgs::Imu> imu_buffer_;
  // computer vision objects
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<VIOInitializer> initializer_;
  // these store the most up to date variables for in between optimizations
  std::unordered_map<double, fuse_variables::Orientation3DStamped::SharedPtr>
      tracker_orientations_;
  std::unordered_map<double, fuse_variables::Position3DStamped::SharedPtr>
      tracker_positions_;
  std::unordered_map<uint64_t, fuse_variables::Position3D::SharedPtr>
      landmark_positions_;
  fuse_core::Graph::ConstSharedPtr graph_;
  bool graph_initialized = false;
  // most recent keyframe timestamp
  ros::Time cur_kf_time_;
  std::string source_ = "VO";
};
}} // namespace beam_models::camera_to_camera
