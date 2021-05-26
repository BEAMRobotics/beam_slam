#include <beam_models/frame_to_frame/imu_preintegration.h>

#include <fuse_core/transaction.h>

#include <beam_utils/math.h>

namespace beam_models { namespace frame_to_frame {

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
  for (size_t i = 0; i < imu_data_buffer_.size(); ++i) imu_data_buffer_.pop();
}

void ImuPreintegration::PopulateBuffer(const sensor_msgs::Imu::ConstPtr& msg) {
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

void ImuPreintegration::SetStart(
    const ros::Time& t_start,
    fuse_variables::Orientation3DStamped::SharedPtr orientation,
    fuse_variables::Position3DStamped::SharedPtr position,
    fuse_variables::VelocityLinear3DStamped::SharedPtr velocity) {
  // adjust imu buffer
  if (imu_data_buffer_.empty()) {
    ROS_FATAL_STREAM("imu buffer must be populated");
  }

  if (t_start.toSec() < imu_data_buffer_.front().t) {
    ROS_FATAL_STREAM("Requested start must have at least imu msg prior");
  }

  while (t_start.toSec() > imu_data_buffer_.front().t) {
    imu_data_buffer_.pop();
  }

  if (imu_data_buffer_.empty()) {
    ROS_FATAL_STREAM("Requested start falls outside of imu buffer");
  }

  // set imu state
  imu_state_i_.InstantiateFuseVariables(t_start);

  if (orientation != nullptr) {
    imu_state_i_.SetOrientation(orientation->data());
  } else {
    imu_state_i_.SetOrientation(1, 0, 0, 0);
  }

  if (position != nullptr) {
    imu_state_i_.SetPosition(position->data());
  } else {
    imu_state_i_.SetPosition(0, 0, 0);
  }

  if (velocity != nullptr) {
    imu_state_i_.SetVelocity(velocity->data());
  } else {
    imu_state_i_.SetVelocity(0, 0, 0);
  }

  imu_state_i_.SetBiasGyroscope(bg_);
  imu_state_i_.SetBiasAcceleration(ba_);

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
                         imu_state_curr.BiasGyroscopeVec(),
                         imu_state_curr.BiasAccelerationVec());
  return imu_state_new;
}

Eigen::Matrix<double, 16, 1> ImuPreintegration::CalculateRelativeChange(
    const ImuState& imu_state_new) {
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
      imu_state_new.BiasGyroscopeVec() - imu_state_i_.BiasGyroscopeVec();
  Eigen::Vector3d bias_accel_delta =
      imu_state_new.BiasAccelerationVec() - imu_state_i_.BiasAccelerationVec();

  Eigen::Matrix<double, 16, 1> delta;
  delta << or_delta.w(), or_delta.vec(), pos_delta, vel_delta, bias_gyro_delta,
      bias_accel_delta;
  return delta;
}

Eigen::Matrix4d ImuPreintegration::GetPose(const ros::Time& t_now) {
  // encapsulate imu measurments between frames
  PreIntegrator pre_integrator_interval;

  if (t_now.toSec() < imu_data_buffer_.front().t)
    ROS_FATAL_STREAM("Requested pose falls outside of imu buffer");

  // Populate integrators
  while (t_now.toSec() > imu_data_buffer_.front().t) {
    pre_integrator_interval.data.emplace_back(imu_data_buffer_.front());
    pre_integrator_ij.data.emplace_back(imu_data_buffer_.front());
    imu_data_buffer_.pop();
  }

  // integrate between frames
  pre_integrator_interval.integrate(
      t_now.toSec(), imu_state_i_.BiasGyroscopeVec(),
      imu_state_i_.BiasAccelerationVec(), false, false);

  // predict state at end of window using integrated imu measurements
  ImuState imu_state_k = PredictState(pre_integrator_interval, imu_state_k_);
  imu_state_k_ = std::move(imu_state_k);

  Eigen::Matrix4d T_WORLD_ISk;
  beam::QuaternionAndTranslationToTransformMatrix(
      imu_state_k_.OrientationQuat(), imu_state_k_.PositionVec(), T_WORLD_ISk);

  return T_WORLD_ISk;
}

beam_constraints::frame_to_frame::ImuState3DStampedTransaction
ImuPreintegration::RegisterNewImuPreintegratedFactor(
    const ros::Time& t_now,
    fuse_variables::Orientation3DStamped::SharedPtr orientation,
    fuse_variables::Position3DStamped::SharedPtr position) {
  beam_constraints::frame_to_frame::ImuState3DStampedTransaction transaction(
      t_now);

  // generate prior constraint at start
  if (first_window_) {
    Eigen::Matrix<double, 15, 15> prior_covariance;
    prior_covariance.setIdentity();
    prior_covariance *= params_.prior_noise;

    transaction.AddImuStatePrior(
        imu_state_i_.Orientation(), imu_state_i_.Position(),
        imu_state_i_.Velocity(), imu_state_i_.BiasGyroscope(),
        imu_state_i_.BiasAcceleration(), prior_covariance,
        "FIRST_IMU_STATE_PRIOR");

    transaction.AddImuStateVariables(
        imu_state_i_.Orientation(), imu_state_i_.Position(),
        imu_state_i_.Velocity(), imu_state_i_.BiasGyroscope(),
        imu_state_i_.BiasAcceleration(), imu_state_i_.Stamp());

    first_window_ = false;
  }

  // Populate integrator
  while (t_now.toSec() > imu_data_buffer_.front().t) {
    pre_integrator_ij.data.emplace_back(imu_data_buffer_.front());
    imu_data_buffer_.pop();
  }

  // integrate between key frames
  pre_integrator_ij.integrate(t_now.toSec(), imu_state_i_.BiasGyroscopeVec(),
                              imu_state_i_.BiasAccelerationVec(), true, true);

  // predict state at end of window using integrated imu measurements
  ImuState imu_state_j = PredictState(pre_integrator_ij, imu_state_i_);

  // update orientation and position of predicted imu state with arguments
  if (orientation != nullptr) {
    imu_state_j.SetOrientation(orientation->data());
  }

  if (position != nullptr) {
    imu_state_j.SetPosition(position->data());
  }

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
      imu_state_j.Velocity(), imu_state_i_.BiasGyroscope(),
      imu_state_j.BiasGyroscope(), imu_state_i_.BiasAcceleration(),
      imu_state_j.BiasAcceleration(), delta_ij, covariance_ij, pre_integrator);

  transaction.AddImuStateVariables(
      imu_state_j.Orientation(), imu_state_j.Position(), imu_state_j.Velocity(),
      imu_state_j.BiasGyroscope(), imu_state_j.BiasAcceleration(),
      imu_state_j.Stamp());

  // move predicted state to previous state
  imu_state_i_ = std::move(imu_state_j);

  ResetPreintegrator();

  return transaction;
}

}}  // namespace beam_models::frame_to_frame
