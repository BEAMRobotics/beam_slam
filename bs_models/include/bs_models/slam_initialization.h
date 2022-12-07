#pragma once

#include <queue>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/macros.h>
#include <fuse_core/throttled_callback.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <beam_calibration/CameraModel.h>
#include <beam_containers/LandmarkContainer.h>

#include <bs_common/bs_msgs.h>
#include <bs_models/vision/visual_map.h>
#include <bs_parameters/models/calibration_params.h>
#include <bs_parameters/models/slam_initialization_params.h>

namespace bs_models {

using namespace bs_common;
using namespace vision;

class SLAMInitialization : public fuse_core::AsyncSensorModel {
public:
  SMART_PTR_DEFINITIONS(SLAMInitialization);

  /**
   * @brief Default Constructor
   */
  SLAMInitialization();

  /**
   * @brief Default Destructor
   */
  ~SLAMInitialization() override = default;

private:
  /**
   * @brief Callback for image processing, this callback will add visual
   * constraints and triangulate new landmarks when required
   * @param[in] msg - The image to process
   */
  void processMeasurements(const CameraMeasurementMsg::ConstPtr& msg);

  /**
   * @brief Callback for imu processing, this will make sure the imu messages
   * are added to the buffer at the correct time
   * @param[in] msg - The imu msg to process
   */
  void processIMU(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief Callback for lidar processing
   * @param[in] msg - The lidar msg to process
   */
  void processLidar(const sensor_msgs::PointCloud2::ConstPtr& msg);

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
   * @brief Unsubscribe to the input topic
   */
  void onStop() override {}

  fuse_core::UUID device_id_; //!< The UUID of this device

  // calibration parameters
  bs_parameters::models::CalibrationParams calibration_params_;

  // loadable parameters
  bs_parameters::models::SLAMInitializationParams slam_initialization_params_;

  // subscribers
  ros::Subscriber visual_measurement_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber lidar_subscriber_;

  // message queues for proper synchronization
  std::queue<CameraMeasurementMsg> image_buffer_;
  std::queue<sensor_msgs::Imu> imu_buffer_;
  std::queue<sensor_msgs::PointCloud2> lidar_buffer_;

  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_containers::LandmarkContainer> landmark_container_;
  std::shared_ptr<VisualMap> visual_map_;
  std::shared_ptr<ImuPreintegration> imu_preint_;
  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer_;

  // callbacks for messages
  using ThrottledMeasurementCallback =
      fuse_core::ThrottledMessageCallback<CameraMeasurementMsg>;
  ThrottledMeasuremenCallback throttled_measurement_callback_;
  using ThrottledIMUCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::Imu>;
  ThrottledIMUCallback throttled_imu_callback_;
  using ThrottledLidarCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::PointCloud2>;
  ThrottledLidarCallback throttled_callback_;

  // extrinsics
  Eigen::Matrix4d T_cam_baselink_;
  Eigen::Matrix4d T_lidar_baselink_;
  Eigen::Matrix4d T_imu_baselink_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
};

} // namespace bs_models
