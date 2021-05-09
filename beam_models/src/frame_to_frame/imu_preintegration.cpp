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
    fuse_variables::Orientation3DStamped::SharedPtr orientation,
    fuse_variables::Position3DStamped::SharedPtr position,
    fuse_variables::VelocityLinear3DStamped::SharedPtr velocity) {
  if (orientation->stamp() != position->stamp() ||
      orientation->stamp() != velocity->stamp() ||
      position->stamp() != velocity->stamp()) {
    ROS_FATAL_STREAM(
        "Fuse variables must share a common time stamp "
        "When initializing source: "
        << params_.source);
  }

  ros::Time t_start = orientation->stamp();

  if (t_start.toSec() > imu_data_buffer_.front().t) {
    int counter = 0;
    while (t_start.toSec() > imu_data_buffer_.at(counter).t) {
      ++counter;
      if (counter == imu_data_buffer_.size()) {
        ROS_FATAL_STREAM(
            "Requested start time falls outside the buffer of imu "
            "measurements.");
        break;
      }
    }

    imu_data_buffer_.erase(imu_data_buffer_.begin(),
                           imu_data_buffer_.begin() + counter);
  }

  imu_state_i_.InstantiateFuseVariables(t_start);
  imu_state_i_.SetOrientation(orientation->data());
  imu_state_i_.SetPosition(position->data());
  imu_state_i_.SetVelocity(velocity->data());
  imu_state_i_.SetBiasGyroscope(bg_);
  imu_state_i_.SetBiasAcceleration(ba_);
}

ImuState ImuPreintegration::PredictState(const PreIntegrator& pre_integrator,
                                         ImuState& imu_state) {
  Eigen::Matrix3d R_i = imu_state.OrientationQuat().toRotationMatrix();
  Eigen::Matrix3d R_j = R_i * pre_integrator.delta.q.matrix();
  Eigen::Vector3d V_j = imu_state.VelocityVec() + g_ * pre_integrator.delta.t +
                        R_i * pre_integrator.delta.v;
  Eigen::Vector3d P_j =
      imu_state.PositionVec() +
      imu_state.VelocityVec() * pre_integrator.delta.t +
      0.5 * g_ * pre_integrator.delta.t * pre_integrator.delta.t +
      R_i * pre_integrator.delta.p;

  Eigen::Quaterniond R_j_quat(R_j);
  ros::Time t_new = imu_state.Stamp() + ros::Duration(pre_integrator.delta.t);
  ImuState new_imu_state(t_new, R_j_quat, P_j, V_j,
                         imu_state.BiasGyroscopeVec(),
                         imu_state.BiasAccelerationVec());
  return new_imu_state;
}

beam_constraints::frame_to_frame::ImuState3DStampedTransaction
ImuPreintegration::RegisterNewImuPreintegratedFactor(
    fuse_variables::Orientation3DStamped::SharedPtr orientation,
    fuse_variables::Position3DStamped::SharedPtr position) {
  ros::Time t_now = orientation->stamp();
  beam_constraints::frame_to_frame::ImuState3DStampedTransaction transaction(
      t_now);

  // ensure consistant time stamps
  if (orientation->stamp() != position->stamp()) {
    ROS_ERROR_STREAM(
        "Fuse variables must share a common time stamp when generating a "
        "transaction for source: "
        << params_.source << "No transaction has been generated.");
    return transaction;
  }

  // get iterator to first time stamp after desired time stamp. This points 
  // to the end of the window
  auto it_after =
      std::find_if(imu_data_buffer_.begin(), imu_data_buffer_.end(),
                   [&t_now](const ImuPreintegration::ImuData& data) {
                     return data.t_ros > t_now;
                   });

  if (std::distance(imu_data_buffer_.begin(), it_after) <= 2) {
    ROS_ERROR_STREAM(
        "Requested time must allow window size of two or more imu "
        "measurements. No transaction has been generated.");
    return transaction;
  }

  if (it_after == imu_data_buffer_.end()) {
    ROS_ERROR_STREAM(
        "Requested time does not exist within imu buffer. No transaction has "
        "been generated.");
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

  // instantiate PreIntegrator class to containerize imu measurments
  std::shared_ptr<PreIntegrator> pre_integrator_ij;
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
  imu_state_j.SetOrientation(orientation->data());
  imu_state_j.SetPosition(position->data());

  // containerize state change as determined by preintegrator class
  Eigen::Matrix<double, 16, 1> delta_ij;
  delta_ij.setZero();
  delta_ij << pre_integrator_ij->delta.q.w(), pre_integrator_ij->delta.q.vec(),
      pre_integrator_ij->delta.p, pre_integrator_ij->delta.v;

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
