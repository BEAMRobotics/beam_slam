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
      .buffer_size = params_.buffer_size,
      .gravitational_acceleration = params_.gravitational_acceleration,
      .initial_imu_acceleration_bias = params_.initial_imu_acceleration_bias,
      .initial_imu_gyroscope_bias = params_.initial_imu_gyroscope_bias};
  imu_preintegration_ =
      std::make_unique<ImuPreintegration>(imu_preintegration_params);

  imu_preintegration_->SetFixedCovariance(params_.imu_noise_covariance);
  imu_preintegration_->ReserveBuffer();
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

  imu_preintegration_->PopulateBuffer(msg);
  imu_preintegration_->PopulateBuffer(msg);

}

}}  // namespace frame_to_frame
