#pragma once
// std
#include <queue>
// ros
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
// beam_slam
#include <beam_models/camera_to_camera/vio_initializer.h>
#include <beam_models/camera_to_camera/visual_map.h>
#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <beam_parameters/models/vio_params.h>
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
   * @brief Registers a frame against the current map
   * @param img_time timestamp of the image to register (must have previously
   * been added to the tracker)
   * @return the transaction containing the frame information
   */
  std::shared_ptr<fuse_core::Transaction>
      registerFrame(const ros::Time& img_time);

  /**
   * @brief Initializes the map with values in the initializer after it has
   * succeeded
   * @return the transaction containing all map init information
   */
  std::shared_ptr<fuse_core::Transaction> initMap();

  /**
   * @brief Converts ros image message to opencv image
   */
  cv::Mat extractImage(const sensor_msgs::Image& msg);

  /**
   * @brief Triangulates a feature track
   * @return the 3d location of the feature
   */
  beam::opt<Eigen::Vector3d> triangulate(beam_cv::FeatureTrack track);

  /**
   * @brief Helper function to get a landmark by id
   * The landmark that will be retrieved will be in imu-world frame, transform
   * to camera before returning
   * @param track feature track of current image
   */
  beam::opt<Eigen::Vector3d> GetLandmark(uint64_t landmark_id);

  /**
   * @brief Helper function to add a new landmark variable to a transaction
   * The landmark being added is in camera coord system, first transform it to
   * imu before adding
   * @param track feature track of current image
   */
  void AddLandmark(const Eigen::Vector3d& p, uint64_t id,
                   std::shared_ptr<fuse_core::Transaction> transaction);

  /**
   * @brief Helper function to get a pose at time t
   * @param track feature track of current image
   */
  beam::opt<Eigen::Matrix4d> GetCameraPose(const ros::Time& stamp);

  /**
   * @brief Helper function to get a pose at time t
   * @param track feature track of current image
   */
  void AddCameraPose(const ros::Time& stamp,
                     const Eigen::Matrix4d& T_WORLD_CAMERA,
                     std::shared_ptr<fuse_core::Transaction> transaction);

  /**
   * @brief Helper function to add a constraint between a landmark and a pose
   * @param track feature track of current image
   */
  void AddConstraint(const ros::Time& img_time, uint64_t lm_id,
                     std::shared_ptr<fuse_core::Transaction> transaction);

protected:
  int img_num_{};
  // loadable camera parameters
  beam_parameters::models::VIOParams params_;
  // topic subscribers and buffers
  ros::Subscriber image_subscriber_;
  ros::Subscriber imu_subscriber_;
  std::queue<sensor_msgs::Image> image_buffer_;
  std::queue<sensor_msgs::Imu> imu_buffer_;
  std::queue<sensor_msgs::Imu> temp_imu_buffer_;
  // computer vision objects
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<VIOInitializer> initializer_;
  std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration> imu_preint_;
  // most recent keyframe timestamp
  ros::Time cur_kf_time_;
  std::string source_ = "VO";
  // landmark/graph stuff
  std::unordered_map<uint64_t, fuse_variables::Position3D::SharedPtr>
      landmark_positions_;
  fuse_core::Graph::ConstSharedPtr graph_;
  std::shared_ptr<VisualMap>
      visual_map_; // replace with graph frame initializer
};

}} // namespace beam_models::camera_to_camera
