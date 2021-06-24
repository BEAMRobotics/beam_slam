#pragma once
// std
#include <queue>
// ros
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
// beam_slam
#include <beam_models/camera_to_camera/initial/lvio_initializer.h>
#include <beam_models/camera_to_camera/visual_map.h>
#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <beam_parameters/models/vio_params.h>
// fuse
#include <fuse_core/async_sensor_model.h>
// libbeam
#include <beam_calibration/CameraModel.h>
#include <beam_cv/geometry/PoseRefinement.h>
#include <beam_cv/tracker/Tracker.h>

namespace beam_models { namespace camera_to_camera {

class VisualInertialOdom : public fuse_core::AsyncSensorModel {
public:
  SMART_PTR_DEFINITIONS(VisualInertialOdom);

  /**
   * @brief Default Constructor
   */
  VisualInertialOdom();

  /**
   * @brief Default Destructor
   */
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
   * @brief Converts ros image message to opencv image
   * @param msg image message to convert to cv mat
   */
  cv::Mat extractImage(const sensor_msgs::Image& msg);

  /**
   * @brief Determines if the current frame is a keyframe
   * @param img_time timestamp of image to check if its a valid keyframe (should
   * be the most recently added image)
   */
  bool IsKeyframe(ros::Time img_time);

protected:
  int img_num_{0};
  // loadable camera parameters
  beam_parameters::models::VIOParams params_;
  // topic subscribers and buffers
  ros::Subscriber image_subscriber_;
  ros::Subscriber imu_subscriber_;
  std::queue<sensor_msgs::Image> image_buffer_;
  std::queue<sensor_msgs::Imu> imu_buffer_;
  std::queue<sensor_msgs::Imu> temp_imu_buffer_;
  // computer vision objects
  std::shared_ptr<beam_cv::PoseRefinement> pose_refiner_;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<beam_models::camera_to_camera::LVIOInitializer> initializer_;
  std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration> imu_preint_;
  // most recent keyframe timestamp
  ros::Time cur_kf_time_ = ros::Time(0);
  std::deque<uint64_t> keyframes_;
  std::string source_ = "VO";
  Eigen::Matrix4d T_imu_cam_;
  // stores all access to graph
  std::shared_ptr<VisualMap> visual_map_;
  ros::Time last_stamp_;
  nav_msgs::Path path_;
  bool set_once = false;
};

}} // namespace beam_models::camera_to_camera
