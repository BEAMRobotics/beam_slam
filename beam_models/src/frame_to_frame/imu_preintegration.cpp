#include <beam_models/frame_to_frame/imu_preintegration.h>

#include <fuse_core/transaction.h>

namespace beam_models { namespace frame_to_frame {

ImuPreintegration::ImuPreintegration(const Params& params) : params_(params) {
  g_ << 0, 0, -params_.gravitational_acceleration;
}

ImuPreintegration::ImuPreintegration(const Params& params,
                                     const Eigen::Vector3d& init_bg,
                                     const Eigen::Vector3d& init_ba)
    : params_(params), bg_(init_bg), ba_(init_ba) {
  g_ << 0, 0, -params_.gravitational_acceleration;
}

void ImuPreintegration::PopulateBuffer(const sensor_msgs::Imu::ConstPtr& msg) {
  ImuData imu_data(msg);
  imu_data_buffer_.emplace_back(imu_data);
}

void ImuPreintegration::PopulateBuffer(const ImuData& imu_data) {
  imu_data_buffer_.emplace_back(imu_data);
}

void ImuPreintegration::SetStart(
    const ros::Time& t_start,
    fuse_variables::Orientation3DStamped::SharedPtr orientation,
    fuse_variables::Position3DStamped::SharedPtr position,
    fuse_variables::VelocityLinear3DStamped::SharedPtr velocity) {
  std::string msg =
      "Requested start time falls outside the buffer of imu measurements.";

  if (t_start.toSec() < imu_data_buffer_.front().t) {
    ROS_FATAL_STREAM(msg);
  }

  if (t_start.toSec() > imu_data_buffer_.front().t) {
    int counter = 0;
    while (t_start.toSec() > imu_data_buffer_.at(counter).t) {
      ++counter;
      if (counter == imu_data_buffer_.size()) {
        ROS_FATAL_STREAM(msg);
        break;
      }
    }

    imu_data_buffer_.erase(imu_data_buffer_.begin(),
                           imu_data_buffer_.begin() + counter);
  }

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
}

ImuState ImuPreintegration::PredictState(const PreIntegrator& pre_integrator,
                                         const ImuState& imu_state) {
  double delta_t = pre_integrator.delta.t;
  Eigen::Matrix3d start_orientation =
      imu_state.OrientationQuat().toRotationMatrix();
  Eigen::Matrix3d end_orientation =
      start_orientation * pre_integrator.delta.q.matrix();
  Eigen::Vector3d end_velocity = imu_state.VelocityVec() + g_ * delta_t +
                                 start_orientation * pre_integrator.delta.v;
  Eigen::Vector3d end_position =
      imu_state.PositionVec() + imu_state.VelocityVec() * delta_t +
      0.5 * g_ * delta_t * delta_t + start_orientation * pre_integrator.delta.p;

  Eigen::Quaterniond end_orientation_quat(end_orientation);
  ros::Time t_new = imu_state.Stamp() + ros::Duration(delta_t);
  ImuState new_imu_state(t_new, end_orientation_quat, end_position,
                         end_velocity, imu_state.BiasGyroscopeVec(),
                         imu_state.BiasAccelerationVec());
  return new_imu_state;
}

beam_constraints::frame_to_frame::ImuState3DStampedTransaction
ImuPreintegration::RegisterNewImuPreintegratedFactor(
    const ros::Time& t_now,
    fuse_variables::Orientation3DStamped::SharedPtr orientation,
    fuse_variables::Position3DStamped::SharedPtr position) {
  beam_constraints::frame_to_frame::ImuState3DStampedTransaction transaction(
      t_now);

  auto it_after =
      std::find_if(imu_data_buffer_.begin(), imu_data_buffer_.end(),
                   [&t_now](const ImuPreintegration::ImuData& data) {
                     return data.t_ros >= t_now;
                   });

  if (std::distance(imu_data_buffer_.begin(), it_after) <= 2) {
    ROS_ERROR_STREAM("Requested time ["
                     << t_now
                     << " sec] must allow window size of two or more imu "
                        "measurements. No transaction has been generated.");
    return transaction;
  }

  if (it_after == imu_data_buffer_.end()) {
    ROS_ERROR_STREAM("Requested time ["
                     << t_now
                     << " sec] does not exist within imu buffer. No "
                        "transaction has been generated.");
    return transaction;
  }

  // generate prior constraint on first imu state
  if (first_window_) {
    Eigen::Matrix<double, 15, 15> imu_state_prior_covariance;
    imu_state_prior_covariance.setIdentity();
    imu_state_prior_covariance *= params_.prior_noise;

    transaction.AddImuStatePrior(
        imu_state_i_.Orientation(), imu_state_i_.Position(),
        imu_state_i_.Velocity(), imu_state_i_.BiasGyroscope(),
        imu_state_i_.BiasAcceleration(), imu_state_prior_covariance,
        params_.source);

    transaction.AddImuStateVariables(
        imu_state_i_.Orientation(), imu_state_i_.Position(),
        imu_state_i_.Velocity(), imu_state_i_.BiasGyroscope(),
        imu_state_i_.BiasAcceleration(), imu_state_i_.Stamp());

    first_window_ = false;
  }

  // instantiate PreIntegrator class to encapsulate imu measurments
  std::shared_ptr<PreIntegrator> pre_integrator_ij =
      std::make_shared<PreIntegrator>();
  pre_integrator_ij->cov_w = params_.cov_gyro_noise;
  pre_integrator_ij->cov_a = params_.cov_accel_noise;
  pre_integrator_ij->cov_bg = params_.cov_gyro_bias;
  pre_integrator_ij->cov_ba = params_.cov_accel_bias;

  // over desired window of imu measurements, perform preintegration
  pre_integrator_ij->data.insert(pre_integrator_ij->data.end(),
                                 imu_data_buffer_.begin(), it_after);
  pre_integrator_ij->integrate(t_now.toSec(), imu_state_i_.BiasGyroscopeVec(),
                               imu_state_i_.BiasAccelerationVec(), true, true);

  // clear buffer of measurements passed to preintegrator class
  imu_data_buffer_.erase(imu_data_buffer_.begin(), it_after);

  // predict state at end of window using integrated imu measurements
  ImuState imu_state_j = PredictState(*pre_integrator_ij, imu_state_i_);

  // update orientation and position of predicted imu state with arguments
  if (orientation != nullptr) {
    imu_state_j.SetOrientation(orientation->data());
  }

  if (position != nullptr) {
    imu_state_j.SetPosition(position->data());
  }

  // containerize state change as determined by preintegrator class
  Eigen::Matrix<double, 16, 1> delta_ij;
  delta_ij << pre_integrator_ij->delta.q.w(), pre_integrator_ij->delta.q.vec(),
      pre_integrator_ij->delta.p, pre_integrator_ij->delta.v, 0, 0, 0, 0, 0, 0;

  // generate relative constraints between imu states
  transaction.AddImuStateConstraint(
      imu_state_i_.Orientation(), imu_state_j.Orientation(),
      imu_state_i_.Position(), imu_state_j.Position(), imu_state_i_.Velocity(),
      imu_state_j.Velocity(), imu_state_i_.BiasGyroscope(),
      imu_state_j.BiasGyroscope(), imu_state_i_.BiasAcceleration(),
      imu_state_j.BiasAcceleration(), delta_ij, pre_integrator_ij->delta.cov);

  transaction.AddImuStateVariables(
      imu_state_j.Orientation(), imu_state_j.Position(), imu_state_j.Velocity(),
      imu_state_j.BiasGyroscope(), imu_state_j.BiasAcceleration(),
      imu_state_j.Stamp());

  // move predicted state to previous state
  imu_state_i_ = std::move(imu_state_j);

  return transaction;
}

}}  // namespace beam_models::frame_to_frame
