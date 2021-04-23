#include <beam_models/frame_to_frame/imu_preintegration.h>

#include <fuse_core/transaction.h>
#include <manif/manif.h>

namespace beam_models {
namespace frame_to_frame {

ImuPreintegration::ImuPreintegration(const Params& params) : params_(params) {
  gravitational_acceleration_ << 0, 0, -params_.gravitational_acceleration;
}

void ImuPreintegration::PopulateBuffer(const sensor_msgs::Imu::ConstPtr& msg) {
  ImuData imu_data;
  imu_data.time = msg->header.stamp;
  imu_data.linear_acceleration[0] = msg->linear_acceleration.x;
  imu_data.linear_acceleration[1] = msg->linear_acceleration.y;
  imu_data.linear_acceleration[2] = msg->linear_acceleration.z;
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

// void
// ImuPreintegration::setFirstFrame(beam_constraints::frame_to_frame::ImuState3DStampedTransaction
// transaction) {
//   R_i_ = fuse_variables::Orientation3DStamped::make_shared(
//       imu_data_buffer_.at(0).time, params_.device_id);
//   V_i_ = fuse_variables::VelocityLinear3DStamped::make_shared(
//       imu_data_buffer_.at(0).time, params_.device_id);
//   P_i_ = fuse_variables::Position3DStamped::make_shared(
//       imu_data_buffer_.at(0).time, params_.device_id);
//   Ba_i_ = beam_variables::ImuBiasStamped::make_shared(
//       imu_data_buffer_.at(0).time, params_.device_id);
//   Bg_i_ = beam_variables::ImuBiasStamped::make_shared(
//       imu_data_buffer_.at(0).time, params_.device_id);

//   Eigen::Quaterniond R_i_quat_init(Eigen::Quaterniond::FromTwoVectors(
//       imu_data_buffer_.at(0).linear_acceleration, Eigen::Vector3d::UnitZ()));

//   R_i_->w() = R_i_quat_init.w();
//   R_i_->x() = R_i_quat_init.x();
//   R_i_->y() = R_i_quat_init.y();
//   R_i_->z() = R_i_quat_init.z();

//   V_i_->x() = 0;
//   V_i_->y() = 0;
//   V_i_->z() = 0;

//   P_i_->x() = 0;
//   P_i_->y() = 0;
//   P_i_->z() = 0;

//   Ba_i_->x() = params_.initial_imu_acceleration_bias;
//   Ba_i_->y() = params_.initial_imu_acceleration_bias;
//   Ba_i_->z() = params_.initial_imu_acceleration_bias;

//   Bg_i_->x() = params_.initial_imu_gyroscope_bias;
//   Bg_i_->y() = params_.initial_imu_gyroscope_bias;
//   Bg_i_->z() = params_.initial_imu_gyroscope_bias;

//   // Generate absolute constraint at origin
//   Eigen::Matrix<double, 10, 1> absolute_imu_state_mean;
//   absolute_imu_state_mean << P_i_->x(), P_i_->y(), P_i_->z(), V_i_->x(),
//       V_i_->y(), V_i_->z(), R_i_->w(), R_i_->x(), R_i_->y(), R_i_->z();
//   fuse_core::Matrix6d absolute_imu_state_covariance =
//       fuse_core::Matrix9d::Identity();

//   auto absolute_imu_state_constraint =
//       beam_constraints::global::AbsoluteImuState3DStampedConstraint::
//           make_shared(params_.source, *P_i_, *V_i_, *R_i_,
//                       absolute_imu_state_mean,
//                       absolute_imu_state_covariance);

//   transaction.AddConstraint(absolute_imu_state_constraint);
// }

void ImuPreintegration::SetFixedCovariance(
    const fuse_core::Matrix6d& covariance) {
  imu_noise_covariance_ = covariance;
  use_fixed_imu_noise_covariance_ = true;
}

beam_constraints::frame_to_frame::ImuState3DStampedTransaction
ImuPreintegration::RegisterNewImuPreintegrationFactor() {
  beam_constraints::frame_to_frame::ImuState3DStampedTransaction transaction(
      imu_data_buffer_.back().time);

  //   // --- State Initialisation

  //   // Initialize fuse variables at the start of the first window
  //   if (first_window_) {
  //     setFirstFrame();
  //     first_window_ = false;
  //   }

  //   // Determine first time, last time, and duration of window
  //   ros::Time t_i = imu_data_buffer_.at(0).time;
  //   ros::Time t_j = imu_data_buffer_.back().time;
  //   double del_t_ij = ros::Duration(t_j - t_i).toSec();

  //   // Declare fuse variables at end of window for relative motion estimation
  //   auto R_j =
  //       fuse_variables::Orientation3DStamped::make_shared(t_j,
  //       params_.device_id);
  //   auto V_j = fuse_variables::VelocityLinear3DStamped::make_shared(
  //       t_j, params_.device_id);
  //   auto P_j =
  //       fuse_variables::Position3DStamped::make_shared(t_j,
  //       params_.device_id);
}

}} // namespace beam_models::frame_to_frame
