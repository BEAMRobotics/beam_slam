#pragma once

#include <beam_models/frame_to_frame/frame_to_frame_sensor_model_base.h>
#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <beam_parameters/models/imu_3d_params.h>

namespace beam_models { namespace frame_to_frame {

class Imu3D
    : public FrameToFrameSensorModelBase<
          sensor_msgs::Imu,
          beam_parameters::models::Imu3DParams,
          beam_constraints::frame_to_frame::PoseWithVelocity3DStampedTransaction> {
public:
  SMART_PTR_DEFINITIONS(Imu3D);

  Imu3D();

  ~Imu3D() override = default;

beam_constraints::frame_to_frame::PoseWithVelocity3DStampedTransaction
      GenerateTransaction(const sensor_msgs::Imu::ConstPtr& msg);

protected:
  void onInit() override;

	void onStart() override;

  void onStop() override;

	std::unique_ptr<ImuPreintegration> imu_preintegration_;

  beam_parameters::models::Imu3DParams params_;
};

}} // namespace beam_models::frame_to_frame