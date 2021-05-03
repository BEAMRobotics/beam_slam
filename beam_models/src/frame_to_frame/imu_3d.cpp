#include <beam_models/frame_to_frame/imu_3d.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(beam_models::frame_to_frame::Imu3D,
                       fuse_core::SensorModel)

namespace beam_models { namespace frame_to_frame {

Imu3D::Imu3D() : FrameToFrameSensorModelBase() {}

void Imu3D::onInit() {
  InitiateBaseClass(private_node_handle_);
  params_.loadExtraParams(private_node_handle_);

  // init imu preintegration
  ImuPreintegration::Params imu_preintegration_params{
      .gravitational_acceleration = params_.gravitational_acceleration};
  imu_preintegration_ =
      std::make_unique<ImuPreintegration>(imu_preintegration_params);
}

void Imu3D::onStart() {
  imu_preintegration_->ClearBuffer();
  subscriber_ = node_handle_.subscribe(
      base_params_->subscriber_topic, params_.queue_size,
      &ThrottledCallback::callback, &throttled_callback_);
};

void Imu3D::onStop() {
  imu_preintegration_->ClearBuffer();
  subscriber_.shutdown();
}

beam_constraints::frame_to_frame::ImuState3DStampedTransaction
Imu3D::GenerateTransaction(const sensor_msgs::Imu::ConstPtr& msg) {
  ROS_DEBUG("Received incoming imu message");

  // need to refactor using Jake's interface

  // imu_preintegration_->PopulateBuffer(msg);
  // if (imu_preintegration_->GetBufferTime() >= params_.max_buffer_time) {
  //   imu_preintegration_->RegisterNewImuPreintegrationFactor();
  //   imu_preintegration_->ClearBuffer();
  // }
}

}}  // namespace frame_to_frame
