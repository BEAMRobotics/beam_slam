#include <beam_models/frame_to_frame/imu_preintegration.h>

#include <fuse_core/transaction.h>
#include <manif/manif.h>

namespace beam_models { namespace frame_to_frame {

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

void ImuPreintegration::SetFixedCovariance(
    const fuse_core::Matrix6d& covariance) {
  imu_noise_covariance_ = covariance;
  use_fixed_imu_noise_covariance_ = true;
}

double ImuPreintegration::GetBufferTime() {
  if (imu_data_buffer_.size() > 1) {
    return ros::Duration(imu_data_buffer_.back().time -
                         imu_data_buffer_.front().time)
        .toSec();
  } else {
    return 0.0;
  }
}

void ImuPreintegration::Integrate(fuse_core::Matrix3d& delta_R_ij,
                                  fuse_core::Vector3d& delta_V_ij,
                                  fuse_core::Vector3d& delta_P_ij,
                                  fuse_core::Matrix9d& Covariance_ij) {
  delta_R_ij.setIdentity();
  delta_V_ij.setZero();
  delta_P_ij.setZero();
  Covariance_ij.setZero();

  Eigen::Matrix<double, 9, 9> A;
  Eigen::Matrix<double, 9, 6> B;
  A.setZero();
  B.setZero();

  for (size_t k = 0; k < imu_data_buffer_.size() - 1; k++) {
    double del_t =
        ros::Duration(imu_data_buffer_[k + 1].time - imu_data_buffer_[k].time)
            .toSec();

    fuse_core::Vector3d correct_accel =
        imu_data_buffer_[k].linear_acceleration -
        imu_state_i_.BiasAccelerationVec();
    fuse_core::Vector3d correct_gyro =
        imu_data_buffer_[k].angular_velocity - imu_state_i_.BiasGyroscopeVec();

    manif::SO3Tangentd del_R_tangent(correct_gyro * del_t);
    manif::SO3d del_R_SO3 = del_R_tangent.exp();

    delta_R_ij *= del_R_SO3.rotation();
    delta_V_ij += delta_R_ij * correct_accel * del_t;
    delta_P_ij += (3 / 2) * delta_R_ij * correct_accel * del_t * del_t;

    manif::SO3Tangentd correct_accel_tangent(correct_accel);
    manif::SO3Tangentd::LieAlg correct_accel_lie = correct_accel_tangent.hat();

    A.block<3, 3>(0, 0) = del_R_SO3.rotation().transpose();
    A.block<3, 3>(3, 0) = -delta_R_ij * correct_accel_lie * del_t;
    A.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
    A.block<3, 3>(6, 0) = -0.5 * delta_R_ij * correct_accel_lie * del_t * del_t;
    A.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * del_t;
    A.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();

    manif::SO3Tangentd correct_gyro_tangent(correct_gyro);

    B.block<3, 3>(0, 0) = correct_gyro_tangent.rjac() * del_t;
    B.block<3, 3>(3, 3) = delta_R_ij * del_t;
    B.block<3, 3>(6, 3) = 0.5 * delta_R_ij * del_t * del_t;

    fuse_core::Matrix6d imu_noise_covariance_k;

    if (use_fixed_imu_noise_covariance_) {
      imu_noise_covariance_k = imu_noise_covariance_;
    } else {
      imu_noise_covariance_k = imu_data_buffer_[k].noise_covariance;
    }

    Covariance_ij = A * Covariance_ij * A.transpose() +
                    B * imu_noise_covariance_k * B.transpose();
  }
}

void ImuPreintegration::PredictState(ImuState& imu_state, const double& del_t,
                                     const fuse_core::Matrix3d& delta_R_ij,
                                     const fuse_core::Vector3d& delta_V_ij,
                                     const fuse_core::Vector3d& delta_P_ij) {
  fuse_core::Matrix3d R_i = imu_state.OrientationQuat().toRotationMatrix();
  fuse_core::Matrix3d R_j = R_i * delta_R_ij;
  fuse_core::Vector3d V_j = imu_state.VelocityVec() +
                            gravitational_acceleration_ * del_t +
                            R_i * delta_V_ij;
  fuse_core::Vector3d P_j =
      imu_state.PositionVec() + imu_state.VelocityVec() * del_t +
      0.5 * gravitational_acceleration_ * del_t * del_t + R_i * delta_P_ij;

  Eigen::Quaterniond R_j_quat(R_j);

  imu_state.SetOrientation(R_j_quat);
  imu_state.SetVelocity(V_j);
  imu_state.SetPosition(P_j);
}

beam_constraints::frame_to_frame::ImuState3DStampedTransaction
ImuPreintegration::RegisterNewImuPreintegrationFactor() {
  // determine first time, last time, and duration of window
  ros::Time t_i = imu_data_buffer_.front().time;
  ros::Time t_j = imu_data_buffer_.back().time;
  double del_t_ij = ros::Duration(t_j - t_i).toSec();

  // build transaction
  beam_constraints::frame_to_frame::ImuState3DStampedTransaction transaction(
      t_j);

  // set first imu state at origin
  if (first_window_) {
    imu_state_i_.InstantiateFuseVariables(t_i);
    imu_state_i_.SetBiasAcceleration(params_.initial_imu_acceleration_bias);
    imu_state_i_.SetBiasGyroscope(params_.initial_imu_gyroscope_bias);

    Eigen::Quaterniond R_i_quat_init(Eigen::Quaterniond::FromTwoVectors(
        imu_data_buffer_.front().linear_acceleration,
        Eigen::Vector3d::UnitZ()));

    imu_state_i_.SetOrientation(R_i_quat_init);

    Eigen::Matrix<double, 15, 15> imu_state_prior_covariance;
    imu_state_prior_covariance.setIdentity();
    imu_state_prior_covariance *= imu_state_prior_noise_;

    transaction.AddImuStatePrior(
        imu_state_i_.Orientation(), imu_state_i_.Velocity(),
        imu_state_i_.Position(), imu_state_i_.BiasAcceleration(),
        imu_state_i_.BiasGyroscope(), imu_state_prior_covariance,
        params_.source);

    first_window_ = false;
  }

  // estimate motion and covariance over window
  fuse_core::Matrix3d delta_R_ij;
  fuse_core::Vector3d delta_V_ij;
  fuse_core::Vector3d delta_P_ij;
  fuse_core::Matrix9d Covariance_ij;
  Integrate(delta_R_ij, delta_V_ij, delta_P_ij, Covariance_ij);

  ImuState imu_state_j(t_j);
  imu_state_j.SetBiasAcceleration(imu_state_i_.BiasAccelerationVec());
  imu_state_j.SetBiasGyroscope(imu_state_i_.BiasGyroscopeVec());
  PredictState(imu_state_j, del_t_ij, delta_R_ij, delta_V_ij, delta_P_ij);
}

}  // namespace frame_to_frame
}  // namespace beam_models
