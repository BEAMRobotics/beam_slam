#include <beam_models/frame_to_frame/imu_preintegration.h>

#include <fuse_core/transaction.h>
#include <manif/manif.h>

#include <beam_constraints/frame_to_frame/relative_pose_3d_stamped_transaction.h>

namespace beam_models { namespace frame_to_frame {

ImuPreintegration::ImuPreintegration(const Params& params) : params_(params) {
  gravitational_acceleration_ << 0, 0, -params_.gravitational_acceleration;
}

void ImuPreintegration::clearBuffers() {
  msg_time_buffer_.clear();
  angular_velocity_buffer_.clear();
  linear_acceleration_buffer_.clear();
  imu_noise_buffer_.clear();
}

void ImuPreintegration::reserveBuffers() {
  msg_time_buffer_.reserve(params_.buffer_size);
  angular_velocity_buffer_.reserve(params_.buffer_size);
  linear_acceleration_buffer_.reserve(params_.buffer_size);
  imu_noise_buffer_.reserve(params_.buffer_size);
}

void ImuPreintegration::populateBuffers(const sensor_msgs::Imu::ConstPtr& msg) {
  ros::Time msg_time = msg->header.stamp;

  fuse_core::Vector3d angular_vel;
  angular_vel << msg->angular_velocity.x, msg->angular_velocity.y,
      msg->angular_velocity.z;

  fuse_core::Vector3d linear_accel;
  linear_accel << msg->linear_acceleration.x, msg->linear_acceleration.y,
      msg->linear_acceleration.z;

  if (!use_fixed_imu_noise_covariance_) {
    fuse_core::Matrix6d imu_noise;
    imu_noise.setZero();
    imu_noise.block<3, 3>(0, 0) << msg->angular_velocity_covariance[0],
        msg->angular_velocity_covariance[1],
        msg->angular_velocity_covariance[2],
        msg->angular_velocity_covariance[3],
        msg->angular_velocity_covariance[4],
        msg->angular_velocity_covariance[5],
        msg->angular_velocity_covariance[6],
        msg->angular_velocity_covariance[7],
        msg->angular_velocity_covariance[8];
    imu_noise.block<3, 3>(3, 3) << msg->linear_acceleration_covariance[0],
        msg->linear_acceleration_covariance[1],
        msg->linear_acceleration_covariance[2],
        msg->linear_acceleration_covariance[3],
        msg->linear_acceleration_covariance[4],
        msg->linear_acceleration_covariance[5],
        msg->linear_acceleration_covariance[6],
        msg->linear_acceleration_covariance[7],
        msg->linear_acceleration_covariance[8];
    imu_noise_buffer_.emplace_back(imu_noise);
  }

  msg_time_buffer_.emplace_back(msg_time);
  angular_velocity_buffer_.emplace_back(angular_vel);
  linear_acceleration_buffer_.emplace_back(linear_accel);
}

void ImuPreintegration::setFirstFrame(const sensor_msgs::Imu::ConstPtr& msg) {

}

void ImuPreintegration::SetFixedCovariance(
    const fuse_core::Matrix6d& covariance) {
  imu_noise_covariance_ = covariance;
  use_fixed_imu_noise_covariance_ = true;
}

}}  // namespace beam_models::frame_to_frame
