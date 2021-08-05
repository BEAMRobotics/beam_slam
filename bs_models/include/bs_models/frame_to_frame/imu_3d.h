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
   * @brief Callback for IMU messages
   * @param[in] msg - The IMU message to process
   */
  void process(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief Callback for transaction generation
   * @param[in] msg - The IMU message to process
   * @param[out] msg - transaction
   */
  fuse_core::Transaction::SharedPtr GenerateTransaction(
      const sensor_msgs::Imu::ConstPtr& msg);

  fuse_core::UUID device_id_;  //!< The UUID of this device
  bool set_start_{true};

  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer_;
  std::unique_ptr<ImuPreintegration> imu_preintegration_;
  bs_parameters::models::Imu3DParams params_;

  ros::Subscriber subscriber_;

  using ImuThrottledCallback =
      fuse_core::ThrottledMessageCallback<sensor_msgs::Imu>;
  ImuThrottledCallback throttled_callback_;

  bs_common::ExtrinsicsLookup& extrinsics_ =
      bs_common::ExtrinsicsLookup::GetInstance();
};

}  // namespace frame_to_frame
}  // namespace bs_models