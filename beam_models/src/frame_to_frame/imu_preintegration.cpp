#include <beam_models/frame_to_frame/imu_preintegration.h>

#include <fuse_core/transaction.h>

#include <beam_utils/math.h>

namespace beam_models {
namespace frame_to_frame {

ImuPreintegration::ImuPreintegration(const Params& params) : params_(params) {
  g_ << 0, 0, -params_.gravitational_acceleration;
  SetPreintegrator();
}

ImuPreintegration::ImuPreintegration(const Params& params,
                                     const Eigen::Vector3d& init_bg,
                                     const Eigen::Vector3d& init_ba)
    : params_(params), bg_(init_bg), ba_(init_ba) {
  g_ << 0, 0, -params_.gravitational_acceleration;
  SetPreintegrator();
}

void ImuPreintegration::ClearBuffer() {
  for (size_t i = 0; i < imu_data_buffer_.size(); i++) imu_data_buffer_.pop();
}

void ImuPreintegration::PopulateBuffer(const sensor_msgs::Imu& msg) {
  ImuData imu_data(msg);
  imu_data_buffer_.push(imu_data);
}

void ImuPreintegration::PopulateBuffer(const ImuData& imu_data) {
  imu_data_buffer_.push(imu_data);
}

void ImuPreintegration::SetPreintegrator() {
  pre_integrator_ij.cov_w = params_.cov_gyro_noise;
  pre_integrator_ij.cov_a = params_.cov_accel_noise;
  pre_integrator_ij.cov_bg = params_.cov_gyro_bias;
  pre_integrator_ij.cov_ba = params_.cov_accel_bias;
}

void ImuPreintegration::ResetPreintegrator() {
  pre_integrator_ij.reset();
  pre_integrator_ij.data.clear();
}

void ImuPreintegration::CheckTime(const ros::Time& t_now) {
  if (t_now.toSec() < imu_data_buffer_.front().t) {
    const std::string error = "Requested time preceeds IMU messages in buffer";
    BEAM_ERROR(error);
    throw std::runtime_error(error);
  }
}

void ImuPreintegration::SetStart(
    const ros::Time& t_start,
    fuse_variables::Orientation3DStamped::SharedPtr orientation,
    fuse_variables::Position3DStamped::SharedPtr position,
    fuse_variables::VelocityLinear3DStamped::SharedPtr velocity) {
  // adjust imu buffer
  while (t_start.toSec() > imu_data_buffer_.front().t) {
    imu_data_buffer_.pop();
  }

  // set imu state
  ImuState imu_state_i(t_start);

  if (orientation != nullptr) {
    imu_state_i.SetOrientation(orientation->data());
  }

  if (position != nullptr) { imu_state_i.SetPosition(position->data()); }

  if (velocity != nullptr) { imu_state_i.SetVelocity(velocity->data()); }

  imu_state_i.SetGyroBias(bg_);
  imu_state_i.SetAccelBias(ba_);

  imu_state_i_ = std::move(imu_state_i);

  // copy start imu state to initialize kth frame between keyframes
  imu_state_k_ = imu_state_i_;
}

ImuState ImuPreintegration::PredictState(const PreIntegrator& pre_integrator,
                                         const ImuState& imu_state_curr) {
  // calculate new states
  double dt = pre_integrator.delta.t;
  Eigen::Matrix3d or_curr = imu_state_curr.OrientationQuat().toRotationMatrix();
  Eigen::Matrix3d or_new_mat = or_curr * pre_integrator.delta.q.matrix();
  Eigen::Vector3d vel_new =
      imu_state_curr.VelocityVec() + g_ * dt + or_curr * pre_integrator.delta.v;
  Eigen::Vector3d pos_new =
      imu_state_curr.PositionVec() + imu_state_curr.VelocityVec() * dt +
      0.5 * g_ * dt * dt + or_curr * pre_integrator.delta.p;

  // instantiate new imu state
  Eigen::Quaterniond or_new(or_new_mat);
  ros::Time t_new = imu_state_curr.Stamp() + ros::Duration(dt);
  ImuState imu_state_new(t_new, or_new, pos_new, vel_new,
                         imu_state_curr.GyroBiasVec(),
                         imu_state_curr.AccelBiasVec());
  return imu_state_new;
}

Eigen::Matrix<double, 16, 1>
    ImuPreintegration::CalculateRelativeChange(const ImuState& imu_state_new) {
  Eigen::Matrix3d or_curr_rot_trans = imu_state_i_.OrientationQuat()
                                          .normalized()
                                          .toRotationMatrix()
                                          .transpose();
  Eigen::Matrix3d or_delta_mat =
      or_curr_rot_trans *
      imu_state_new.OrientationQuat().normalized().toRotationMatrix();
  Eigen::Quaterniond or_delta(or_delta_mat);
  Eigen::Vector3d pos_delta = or_curr_rot_trans * (imu_state_new.PositionVec() -
                                                   imu_state_i_.PositionVec());
  Eigen::Vector3d vel_delta = or_curr_rot_trans * (imu_state_new.VelocityVec() -
                                                   imu_state_i_.VelocityVec());
  Eigen::Vector3d bias_gyro_delta =
      imu_state_new.GyroBiasVec() - imu_state_i_.GyroBiasVec();
  Eigen::Vector3d bias_accel_delta =
      imu_state_new.AccelBiasVec() - imu_state_i_.AccelBiasVec();

  Eigen::Matrix<double, 16, 1> delta;
  delta << or_delta.w(), or_delta.vec(), pos_delta, vel_delta, bias_gyro_delta,
      bias_accel_delta;
  return delta;
}

Eigen::Matrix4d ImuPreintegration::GetPose(const ros::Time& t_now) {
  // encapsulate imu measurments between frames
  PreIntegrator pre_integrator_interval;

  // check requested time
  CheckTime(t_now);

  // Populate integrators
  while (t_now.toSec() > imu_data_buffer_.front().t) {
    pre_integrator_interval.data.emplace_back(imu_data_buffer_.front());
    pre_integrator_ij.data.emplace_back(imu_data_buffer_.front());
    imu_data_buffer_.pop();
  }
  // integrate between frames
  pre_integrator_interval.integrate(t_now.toSec(), imu_state_i_.GyroBiasVec(),
                                    imu_state_i_.AccelBiasVec(), false, false);

  // predict state at end of window using integrated imu measurements
  ImuState imu_state_k = PredictState(pre_integrator_interval, imu_state_k_);
  imu_state_k_ = std::move(imu_state_k);

  Eigen::Matrix4d T_WORLD_IMU;
  beam::QuaternionAndTranslationToTransformMatrix(
      imu_state_k_.OrientationQuat(), imu_state_k_.PositionVec(), T_WORLD_IMU);

  return T_WORLD_IMU;
}

beam_constraints::frame_to_frame::ImuState3DStampedTransaction
    ImuPreintegration::RegisterNewImuPreintegratedFactor(
        const ros::Time& t_now,
        fuse_variables::Orientation3DStamped::SharedPtr orientation,
        fuse_variables::Position3DStamped::SharedPtr position) {
  beam_constraints::frame_to_frame::ImuState3DStampedTransaction transaction(
      t_now);

  // check requested time
  CheckTime(t_now);    

  // generate prior constraint at start
  if (first_window_) {
    Eigen::Matrix<double, 15, 15> prior_covariance;
    prior_covariance.setIdentity();
    prior_covariance *= params_.prior_noise;

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

  // Populate integrator
  while (t_now.toSec() > imu_data_buffer_.front().t) {
    pre_integrator_ij.data.emplace_back(imu_data_buffer_.front());
    imu_data_buffer_.pop();
  }

  // integrate between key frames
  pre_integrator_ij.integrate(t_now.toSec(), imu_state_i_.GyroBiasVec(),
                              imu_state_i_.AccelBiasVec(), true, true);

  // predict state at end of window using integrated imu measurements
  ImuState imu_state_j = PredictState(pre_integrator_ij, imu_state_i_);

  // update orientation and position of predicted imu state with arguments
  if (orientation != nullptr) {
    imu_state_j.SetOrientation(orientation->data());
  }

  if (position != nullptr) { imu_state_j.SetPosition(position->data()); }

  // calculate relative change in imu state between key frames
  auto delta_ij = CalculateRelativeChange(imu_state_j);

  // Determine covariance and ensure non-zero
  Eigen::Matrix<double, 15, 15> covariance_ij{pre_integrator_ij.delta.cov};
  if (covariance_ij.isZero(1e-9)) {
    covariance_ij.setIdentity();
    covariance_ij *= params_.prior_noise;
  }

  // make preintegrator a shared pointer for constraint
  auto pre_integrator = std::make_shared<PreIntegrator>(pre_integrator_ij);

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

  // move predicted state to previous state
  imu_state_i_ = std::move(imu_state_j);

  // copy state to kth frame
  imu_state_k_ = imu_state_i_;

  ResetPreintegrator();

  return transaction;
}

}  // namespace frame_to_frame
}  // namespace beam_models
