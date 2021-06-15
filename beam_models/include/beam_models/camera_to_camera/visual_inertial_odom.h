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
   * @brief Registers a frame against the current map
   * @param img_time timestamp of the image to register (must have previously
   * been added to the tracker)
   * @return the transaction containing the frame information
   */
  std::shared_ptr<fuse_core::Transaction>
      registerFrame(const cv::Mat& image, const ros::Time& img_time);

  /**
   * @brief Initializes the map with values in the initializer after it has
   * succeeded
   * @return the transaction containing all map init information
   */
  std::shared_ptr<fuse_core::Transaction> initMap();

  /**
   * @brief Initializes the imu preint object with values form the
   * initialization
   */
  void initIMU();

  /**
   * @brief Converts ros image message to opencv image
   */
  cv::Mat extractImage(const sensor_msgs::Image& msg);

  /**
   * @brief Triangulates a feature track
   * @param feature track to triangulate
   * @return the 3d location of the feature
   */
  beam::opt<Eigen::Vector3d> triangulate(beam_cv::FeatureTrack track);

  /**
   * @brief Triangulates a feature track
   * @param feature track to triangulate
   * @return the 3d location of the feature
   */
  void addIMUConstraint(ros::Time keyframe_time,
                        std::shared_ptr<fuse_core::Transaction> transaction);

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
  std::shared_ptr<VIOInitializer> initializer_;
  std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration> imu_preint_;
  // most recent keyframe timestamp
  ros::Time cur_kf_time_;
  std::deque<uint64_t> keyframes_;
  std::map<uint64_t, cv::Mat> kf_images_;
  std::string source_ = "VO";
  Eigen::Matrix4d T_imu_cam_;
  // stores all access to graph
  std::shared_ptr<VisualMap> visual_map_;
};

}} // namespace beam_models::camera_to_camera
