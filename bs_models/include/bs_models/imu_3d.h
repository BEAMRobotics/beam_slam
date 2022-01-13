#pragma once

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/throttled_callback.h>
#include <fuse_core/uuid.h>

#include <bs_common/extrinsics_lookup_online.h>
#include <bs_constraints/relative_pose/relative_pose_transaction_base.h>
#include <bs_models/frame_initializers/frame_initializers.h>
#include <bs_models/imu/imu_preintegration.h>
#include <bs_parameters/models/imu_3d_params.h>
#include <bs_parameters/models/calibration_params.h>

namespace bs_models {

class Imu3D : public fuse_core::AsyncSensorModel {
 public:
  SMART_PTR_DEFINITIONS(Imu3D);

  /**
   * @brief Default constructor
   */
  Imu3D();

  /**
   * @brief Destructor
   */
  virtual ~Imu3D() = default;

 private:
  /**
   * @brief Perform required initialization for the sensor model
   */
  void onInit() override;

  /**
   * @brief Subscribe to the input topic to start sending transactions to the
   * optimizer
   */
  void onStart() override;

  /**
   * @brief Unsubscribe from the input topic to stop sending transactions to the
   * optimizer
   */
  void onStop() override;

  /**
   * @brief Callback for IMU messages
   * @param msg the IMU message to process
   */
  void process(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief Gets estimated pose of imu wrt world frame using frame initializer
   * @param R_WORLD_IMU reference to orientation result
   * @param t_WORLD_IMU reference to position result
   * @param t_now stamp of requested pose
   * @return true if pose lookup was successful
   */
  bool GetEstimatedPose(
      fuse_variables::Orientation3DStamped::SharedPtr& R_WORLD_IMU,
      fuse_variables::Position3DStamped::SharedPtr& t_WORLD_IMU,
      const ros::Time& time);

  // The UUID of this device
  fuse_core::UUID device_id_;

  // timing
  bool set_start_{true};
  ros::Duration t_lag_;
  ros::Duration t_elapsed_;
  ros::Time t_prev_;
  std::queue<ros::Time> t_buffer_;

  // initialization
  fuse_variables::VelocityLinear3DStamped::SharedPtr init_velocity_;
  Eigen::Vector3d init_gyro_bias_;
  Eigen::Vector3d init_accel_bias_;

  // Frame-to-frame
  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer_;
  std::unique_ptr<ImuPreintegration> imu_preintegration_;
  bs_parameters::models::Imu3DParams params_;
  bs_parameters::models::CalibrationParams calibration_params_;

  // Subscriber/Callback
  ros::Subscriber subscriber_;
  using ThrottledCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::Imu>;
  ThrottledCallback throttled_callback_;

  // Extrinsics
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
};

}  // namespace bs_models
