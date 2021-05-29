#pragma once

#include <beam_models/frame_to_frame/frame_to_frame_sensor_model_base.h>
#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <beam_parameters/models/imu_3d_params.h>

namespace beam_models {
namespace frame_to_frame {

class Imu3D
    : public FrameToFrameSensorModelBase<
          sensor_msgs::Imu, beam_parameters::models::Imu3DParams,
          beam_constraints::frame_to_frame::ImuState3DStampedTransaction> {
 public:
  SMART_PTR_DEFINITIONS(Imu3D);

  /**
   * @brief Default constructor
   */
  Imu3D();

  /**
   * @brief Destructor
   */
  ~Imu3D() override = default;

  /**
   * @brief Callback for transaction generation
   * @param msg - The IMU message to process
   */
  beam_constraints::frame_to_frame::ImuState3DStampedTransaction
  GenerateTransaction(const sensor_msgs::Imu::ConstPtr& msg);

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

  double elapsed_window_time_{0};
  bool set_start_{true};
  ros::Time t_prev_;

  std::unique_ptr<ImuPreintegration> imu_preintegration_;
  beam_parameters::models::Imu3DParams params_;
};

}  // namespace frame_to_frame
}  // namespace beam_models