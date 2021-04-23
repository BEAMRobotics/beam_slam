#include <beam_models/frame_to_frame/imu_3d.h>

#include <numeric>

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

  ImuPreintegration::Params imu_preintegration_params{
      .buffer_size = params_.buffer_size,
      .gravitational_acceleration = params_.gravitational_acceleration,
      .initial_imu_acceleration_bias = params_.initial_imu_acceleration_bias,
      .initial_imu_gyroscope_bias = params_.initial_imu_gyroscope_bias};
  imu_preintegration_ =
      std::make_unique<ImuPreintegration>(imu_preintegration_params);

  // set covariance if not set to zero in config
  if (std::accumulate(params_.imu_noise_diagonal.begin(),
                      params_.imu_noise_diagonal.end(), 0.0) > 0) {
    fuse_core::Matrix6d covariance;
    covariance.setIdentity();
    for (int i = 0; i < 6; i++) {
      covariance(i, i) = params_.imu_noise_diagonal[i];
    }
    imu_preintegration_->SetFixedCovariance(covariance);
  }
  imu_preintegration_->reserveBuffer();
}

void Imu3D::onStart() {
  imu_preintegration_->clearBuffer();
  subscriber_ = node_handle_.subscribe(
      base_params_->subscriber_topic, params_.queue_size,
      &ThrottledCallback::callback, &throttled_callback_);
};

void Imu3D::onStop() {
  imu_preintegration_->clearBuffer();
  subscriber_.shutdown();
}

beam_constraints::frame_to_frame::ImuState3DStampedTransaction
Imu3D::GenerateTransaction(const sensor_msgs::Imu::ConstPtr& msg) {
  ROS_DEBUG("Received incoming imu message");

  imu_preintegration_->populateBuffer(msg);

  if (imu_preintegration_->getBufferSize() == params_.buffer_size) {
    // perform Imu preintegration
  }

  // build transaction of preintegrated imu measurements
}

}}  // namespace beam_models::frame_to_frame
 
