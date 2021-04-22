#include <beam_models/frame_to_frame/imu_preintegration.h>

#include <fuse_core/transaction.h>
#include <manif/manif.h>


namespace beam_models { namespace frame_to_frame {

ImuPreintegration::ImuPreintegration(const Params& params) : params_(params) {
  gravitational_acceleration_ << 0, 0, -params_.gravitational_acceleration;
}

void ImuPreintegration::populateBuffer(const sensor_msgs::Imu::ConstPtr& msg) {

	ImuData imu_data;
	imu_data.time = msg->header.stamp;
	imu_data.linear_acceleration[0] = msg->angular_velocity.x;
	imu_data.linear_acceleration[1] = msg->angular_velocity.y;
  imu_data.linear_acceleration[2] = msg->angular_velocity.z;
	imu_data.angular_velocity[0] = msg->angular_velocity.x;
	imu_data.angular_velocity[1] = msg->angular_velocity.y;
  imu_data.angular_velocity[2] = msg->angular_velocity.z;

  if (!use_fixed_imu_noise_covariance_) {
		imu_data.noise_covariance.block<3, 3>(0, 0) 
				<< msg->angular_velocity_covariance[0],
        msg->angular_velocity_covariance[1],
        msg->angular_velocity_covariance[2],
        msg->angular_velocity_covariance[3],
        msg->angular_velocity_covariance[4],
        msg->angular_velocity_covariance[5],
        msg->angular_velocity_covariance[6],
        msg->angular_velocity_covariance[7],
        msg->angular_velocity_covariance[8];
    imu_data.noise_covariance.block<3, 3>(3, 3) 
				<< msg->linear_acceleration_covariance[0],
        msg->linear_acceleration_covariance[1],
        msg->linear_acceleration_covariance[2],
        msg->linear_acceleration_covariance[3],
        msg->linear_acceleration_covariance[4],
        msg->linear_acceleration_covariance[5],
        msg->linear_acceleration_covariance[6],
        msg->linear_acceleration_covariance[7],
        msg->linear_acceleration_covariance[8];
  }
		
	imu_data_buffer_.emplace_back(imu_data);
}

void ImuPreintegration::setFirstFrame(const sensor_msgs::Imu::ConstPtr& msg) {

}

void ImuPreintegration::SetFixedCovariance(
    const fuse_core::Matrix6d& covariance) {
  imu_noise_covariance_ = covariance;
  use_fixed_imu_noise_covariance_ = true;
}

}}  // namespace beam_models::frame_to_frame
