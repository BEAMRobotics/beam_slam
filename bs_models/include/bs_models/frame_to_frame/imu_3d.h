#pragma once

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/throttled_callback.h>
#include <fuse_core/uuid.h>

#include <bs_constraints/frame_to_frame/frame_to_frame_transaction_base.h>
#include <bs_common/extrinsics_lookup.h>
#include <bs_models/frame_initializers/frame_initializers.h>
#include <bs_models/frame_to_frame/imu_preintegration.h>
#include <bs_parameters/models/imu_3d_params.h>

namespace bs_models {
namespace frame_to_frame {

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

 protected:
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
   * @param[in] msg - The IMU message to process
   */
  void process(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief Gets estimated pose of imu wrt world frame using frame initializer
   * @param[out] R_WORLD_IMU - reference to orientation result
   * @param[out] t_WORLD_IMU - reference to position result
   * @param[in] t_now - stamp of requested pose
   * @return true if pose lookup was successful
   */
  bool GetPose(fuse_variables::Orientation3DStamped::SharedPtr& R_WORLD_IMU,
               fuse_variables::Position3DStamped::SharedPtr& t_WORLD_IMU,
               const ros::Time& time);

  // The UUID of this device
  fuse_core::UUID device_id_;

  // timing parameters
  bool set_start_{true};
  ros::Duration t_elapsed_;
  ros::Time t_prev_;

  // Frame-to-frame objects
  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer_;
  std::unique_ptr<ImuPreintegration> imu_preintegration_;
  bs_parameters::models::Imu3DParams params_;

  // Subscriber/Callback objects
  ros::Subscriber subscriber_;
  using ImuThrottledCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::Imu>;
  ImuThrottledCallback throttled_callback_;

  // Extrinsics
  bs_common::ExtrinsicsLookup& extrinsics_ =
      bs_common::ExtrinsicsLookup::GetInstance();
};

}  // namespace frame_to_frame
}  // namespace bs_models