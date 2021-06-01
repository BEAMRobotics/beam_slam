#include <beam_models/frame_to_frame/imu_3d.h>

#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(beam_models::frame_to_frame::Imu3D,
                       fuse_core::SensorModel)

namespace beam_models {
namespace frame_to_frame {

Imu3D::Imu3D() : FrameToFrameSensorModelBase() {}

void Imu3D::onInit() {
  InitiateBaseClass(private_node_handle_);
  params_.loadExtraParams(private_node_handle_);

  // init imu preintegration
  ImuPreintegration::Params imu_preintegration_params{
      .gravitational_acceleration = params_.gravitational_acceleration,
      .prior_noise = params_.prior_noise,
      .cov_gyro_noise = params_.cov_gyro_noise,
      .cov_accel_noise = params_.cov_accel_noise,
      .cov_gyro_bias = params_.cov_gyro_bias,
      .cov_accel_bias = params_.cov_accel_bias,
      .source = name()};
  imu_preintegration_ =
      std::make_unique<ImuPreintegration>(imu_preintegration_params);
}

void Imu3D::onStart() {
  subscriber_ = node_handle_.subscribe(
      base_params_->subscriber_topic, params_.queue_size,
      &ThrottledCallback::callback, &throttled_callback_);
};

beam_constraints::frame_to_frame::ImuState3DStampedTransaction
Imu3D::GenerateTransaction(const sensor_msgs::Imu::ConstPtr& msg) {
  ROS_DEBUG("Received incoming imu message");
  ros::Time t_now = msg->header.stamp;

  // set start IMU state to Orientation = I, Position = 0, Velocity = 0
  if (set_start_) {
    imu_preintegration_->SetStart(t_now);
    set_start_ = false;
    t_prev_ = t_now;
  }

  // increment elapsed window time 
  elapsed_window_time_ += t_now.toSec() - t_prev_.toSec();
  t_prev_ = t_now;

  // populate imu buffer
  imu_preintegration_->PopulateBuffer(msg);

  // generate transaction
  if (elapsed_window_time_ >= params_.window_time) {
    elapsed_window_time_ = 0;
    return imu_preintegration_->RegisterNewImuPreintegratedFactor(t_now);
  } else {
    return beam_constraints::frame_to_frame::ImuState3DStampedTransaction(
        t_now);
  }
}

}  // namespace frame_to_frame
}  // namespace beam_models
