#include <bs_models/frame_to_frame/imu_preintegration.h>

#include <fuse_core/transaction.h>

#include <beam_utils/math.h>

namespace bs_models {
namespace frame_to_frame {

ImuPreintegration::ImuPreintegration(const Params& params) : params_(params) {
  CheckParameters();
  SetPreintegrator();
}

ImuPreintegration::ImuPreintegration(const Params& params,
                                     const Eigen::Vector3d& init_bg,
                                     const Eigen::Vector3d& init_ba)
    : params_(params), bg_(init_bg), ba_(init_ba) {
  CheckParameters();
  SetPreintegrator();
}

void ImuPreintegration::ClearBuffer() {
  for (size_t i = 0; i < imu_data_buffer_.size(); i++) imu_data_buffer_.pop();
}

void ImuPreintegration::AddToBuffer(const sensor_msgs::Imu& msg) {
  bs_common::IMUData imu_data(msg);
  imu_data_buffer_.push(imu_data);
}

void ImuPreintegration::AddToBuffer(const bs_common::IMUData& imu_data) {
  imu_data_buffer_.push(imu_data);
}

void ImuPreintegration::CheckParameters() {
  if (params_.cov_prior_noise <= 0) {
    BEAM_ERROR("Prior noise on IMU state must be positive");
    throw std::invalid_argument{"Inputs to ImuPreintegration invalid."};
  }
}

void ImuPreintegration::SetPreintegrator() {
  pre_integrator_ij.cov_w = params_.cov_gyro_noise;
  pre_integrator_ij.cov_a = params_.cov_accel_noise;
  pre_integrator_ij.cov_bg = params_.cov_gyro_bias;
  pre_integrator_ij.cov_ba = params_.cov_accel_bias;
}

void ImuPreintegration::ResetPreintegrator() {
  pre_integrator_ij.Reset();
  pre_integrator_ij.data.clear();
}

void ImuPreintegration::SetStart(
    const ros::Time& t_start,
    fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU,
    fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU,
    fuse_variables::VelocityLinear3DStamped::SharedPtr velocity) {
  // adjust IMU buffer
  while (t_start > imu_data_buffer_.front().t) {
    imu_data_buffer_.pop();
  }

  // set IMU state
  bs_common::ImuState imu_state_i(t_start);

  if (R_WORLD_IMU) {
    imu_state_i.SetOrientation(R_WORLD_IMU->data());
  }

  if (t_WORLD_IMU) {
    imu_state_i.SetPosition(t_WORLD_IMU->data());
  }

  if (velocity) {
    imu_state_i.SetVelocity(velocity->data());
  }

  imu_state_i.SetGyroBias(bg_);
  imu_state_i.SetAccelBias(ba_);

  imu_state_i_ = std::move(imu_state_i);

  // copy start IMU state to initialize kth frame between keyframes
  imu_state_k_ = imu_state_i_;
}

bs_common::ImuState ImuPreintegration::PredictState(
    const bs_common::PreIntegrator& pre_integrator,
    const bs_common::ImuState& imu_state_curr, const ros::Time& t_now) {
  // get commonly used variables
  double dt = pre_integrator.delta.t.toSec();
  const Eigen::Matrix3d& q_curr = imu_state_curr.OrientationMat();

  // predict new states
  Eigen::Quaterniond q_new(q_curr * pre_integrator.delta.q.matrix());
  Eigen::Vector3d v_new = imu_state_curr.VelocityVec() + GRAVITY_WORLD * dt +
                          q_curr * pre_integrator.delta.v;
  Eigen::Vector3d p_new =
      imu_state_curr.PositionVec() + imu_state_curr.VelocityVec() * dt +
      0.5 * GRAVITY_WORLD * dt * dt + q_curr * pre_integrator.delta.p;

  // set time
  ros::Time t_new = imu_state_curr.Stamp() + pre_integrator.delta.t;
  if (t_now != ros::Time(0)) {
    t_new = t_now;
  }

  // return predicted IMU state
  bs_common::ImuState imu_state_new(t_new, q_new, p_new, v_new,
                                    imu_state_curr.GyroBiasVec(),
                                    imu_state_curr.AccelBiasVec());
  return imu_state_new;
}

Eigen::Matrix<double, 16, 1> ImuPreintegration::CalculateRelativeChange(
    const bs_common::ImuState& imu_state_new) {
  // get commonly used variables
  const Eigen::Matrix3d& q_curr_transpose =
      imu_state_i_.OrientationMat().transpose();

  // calculate relative change
  Eigen::Quaterniond q_delta(q_curr_transpose * imu_state_new.OrientationMat());
  Eigen::Vector3d p_delta = q_curr_transpose * (imu_state_new.PositionVec() -
                                                imu_state_i_.PositionVec());
  Eigen::Vector3d v_delta = q_curr_transpose * (imu_state_new.VelocityVec() -
                                                imu_state_i_.VelocityVec());
  Eigen::Vector3d bias_gyro_delta =
      imu_state_new.GyroBiasVec() - imu_state_i_.GyroBiasVec();
  Eigen::Vector3d bias_accel_delta =
      imu_state_new.AccelBiasVec() - imu_state_i_.AccelBiasVec();

  // return delta
  Eigen::Matrix<double, 16, 1> delta;
  delta << q_delta.w(), q_delta.vec(), p_delta, v_delta, bias_gyro_delta,
      bias_accel_delta;
  return delta;
}

bool ImuPreintegration::GetPose(Eigen::Matrix4d& T_WORLD_IMU,
                                const ros::Time& t_now) {
  // encapsulate IMU measurements between frames
  bs_common::PreIntegrator pre_integrator_interval;

  // check requested time
  if (t_now < imu_data_buffer_.front().t) {
    return false;
  }

  // Populate integrators
  while (t_now > imu_data_buffer_.front().t) {
    pre_integrator_interval.data.emplace_back(imu_data_buffer_.front());
    pre_integrator_ij.data.emplace_back(imu_data_buffer_.front());
    imu_data_buffer_.pop();
  }
  // integrate between frames
  pre_integrator_interval.Integrate(t_now, imu_state_i_.GyroBiasVec(),
                                    imu_state_i_.AccelBiasVec(), false, false);

  // predict state at end of window using integrated IMU measurements
  bs_common::ImuState imu_state_k =
      PredictState(pre_integrator_interval, imu_state_k_, t_now);
  imu_state_k_ = std::move(imu_state_k);

  // populate transformation matrix
  beam::QuaternionAndTranslationToTransformMatrix(
      imu_state_k_.OrientationQuat(), imu_state_k_.PositionVec(), T_WORLD_IMU);

  return true;
}

fuse_core::Transaction::SharedPtr
ImuPreintegration::RegisterNewImuPreintegratedFactor(
    const ros::Time& t_now,
    fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU,
    fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU) {
  bs_constraints::frame_to_frame::ImuState3DStampedTransaction transaction(
      t_now);

  // check requested time
  if (t_now < imu_data_buffer_.front().t) {
    return nullptr;
  }

  // generate prior constraint at start
  if (first_window_) {
    Eigen::Matrix<double, 15, 15> prior_covariance;
    prior_covariance.setIdentity();
    prior_covariance *= params_.cov_prior_noise;

    transaction.AddImuStatePrior(
        imu_state_i_.Orientation(), imu_state_i_.Position(),
        imu_state_i_.Velocity(), imu_state_i_.GyroBias(),
        imu_state_i_.AccelBias(), prior_covariance, "FIRST_IMU_STATE_PRIOR");

    transaction.AddImuStateVariables(
        imu_state_i_.Orientation(), imu_state_i_.Position(),
        imu_state_i_.Velocity(), imu_state_i_.GyroBias(),
        imu_state_i_.AccelBias(), imu_state_i_.Stamp());

    first_window_ = false;
  }

  // populate integrator
  while (t_now > imu_data_buffer_.front().t && !imu_data_buffer_.empty()) {
    pre_integrator_ij.data.emplace_back(imu_data_buffer_.front());
    imu_data_buffer_.pop();
  }

  // integrate between key frames
  pre_integrator_ij.Integrate(t_now, imu_state_i_.GyroBiasVec(),
                              imu_state_i_.AccelBiasVec(), true, true);

  // predict state at end of window using integrated imu measurements
  bs_common::ImuState imu_state_j =
      PredictState(pre_integrator_ij, imu_state_i_, t_now);

  // calculate relative change in imu state between key frames
  auto delta_ij = CalculateRelativeChange(imu_state_j);

  // Determine covariance and ensure non-zero
  Eigen::Matrix<double, 15, 15> covariance_ij{pre_integrator_ij.delta.cov};
  if (covariance_ij.isZero(1e-9)) {
    covariance_ij.setIdentity();
    covariance_ij *= params_.cov_prior_noise;
  }

  // make preintegrator a shared pointer for constraint
  auto pre_integrator =
      std::make_shared<bs_common::PreIntegrator>(pre_integrator_ij);

  // generate relative constraints between key frames
  transaction.AddImuStateConstraint(
      imu_state_i_.Orientation(), imu_state_j.Orientation(),
      imu_state_i_.Position(), imu_state_j.Position(), imu_state_i_.Velocity(),
      imu_state_j.Velocity(), imu_state_i_.GyroBias(), imu_state_j.GyroBias(),
      imu_state_i_.AccelBias(), imu_state_j.AccelBias(), delta_ij,
      covariance_ij, pre_integrator);

  transaction.AddImuStateVariables(
      imu_state_j.Orientation(), imu_state_j.Position(), imu_state_j.Velocity(),
      imu_state_j.GyroBias(), imu_state_j.AccelBias(), imu_state_j.Stamp());

  // update orientation and position of predicted imu state with arguments
  if (R_WORLD_IMU && t_WORLD_IMU) {
    imu_state_j.SetOrientation(R_WORLD_IMU->data());
    imu_state_j.SetPosition(t_WORLD_IMU->data());
  }

  // move predicted state to previous state
  imu_state_i_ = std::move(imu_state_j);

  // copy state to kth frame
  imu_state_k_ = imu_state_i_;

  ResetPreintegrator();

  return transaction.GetTransaction();
}

}  // namespace frame_to_frame
}  // namespace bs_models
