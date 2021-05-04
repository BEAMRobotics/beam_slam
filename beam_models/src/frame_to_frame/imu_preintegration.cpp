#include <beam_models/frame_to_frame/imu_preintegration.h>

#include <fuse_core/transaction.h>
#include <manif/manif.h>

namespace beam_models { namespace frame_to_frame {

ImuPreintegration::ImuPreintegration(const Params& params) : params_(params) {
  gravitational_acceleration_ << 0, 0, -params_.gravitational_acceleration;
}

ImuPreintegration::ImuPreintegration(const Params& params,
                                     const Eigen::Vector3d& init_ba,
                                     const Eigen::Vector3d& init_bg)
    : params_(params), ba_(init_ba), bg_(init_bg) {
  gravitational_acceleration_ << 0, 0, -params_.gravitational_acceleration;
}

void ImuPreintegration::PopulateBuffer(const sensor_msgs::Imu::ConstPtr& msg) {
  ImuData imu_data;
  imu_data.time = msg->header.stamp;
  imu_data.angular_velocity[0] = msg->angular_velocity.x;
  imu_data.angular_velocity[1] = msg->angular_velocity.y;
  imu_data.angular_velocity[2] = msg->angular_velocity.z;
  imu_data.linear_acceleration[0] = msg->linear_acceleration.x;
  imu_data.linear_acceleration[1] = msg->linear_acceleration.y;
  imu_data.linear_acceleration[2] = msg->linear_acceleration.z;
  imu_data_buffer_.emplace_back(imu_data);
}

void ImuPreintegration::PopulateBuffer(const ImuData& imu_data) {
  imu_data_buffer_.emplace_back(imu_data);
};

void ImuPreintegration::ResetMotionEstimate() {
  delta.t = 0;
  delta.q.setIdentity();
  delta.p.setZero();
  delta.v.setZero();
  delta.cov.setZero();
  jacobian.dq_dbg.setZero();
  jacobian.dp_dbg.setZero();
  jacobian.dp_dba.setZero();
  jacobian.dv_dbg.setZero();
  jacobian.dv_dba.setZero();
}

void ImuPreintegration::SetStart(const ros::Time& t_start, 
                                 const Eigen::Quaterniond& orientation,
                                 const Eigen::Vector3d& velocity,
                                 const Eigen::Vector3d& position) { 
  if (t_start.toSec() > imu_data_buffer_.front().time.toSec()) {
    int counter = 0;
    while (t_start.toSec() > imu_data_buffer_.at(counter).time.toSec()) {
      counter++;
      if (counter == imu_data_buffer_.size()) {
        // throw error
        break;
      }
    }

    imu_data_buffer_.erase(imu_data_buffer_.begin(),
                           imu_data_buffer_.begin() + counter);
  }

  imu_state_i_.InstantiateFuseVariables(t_start);
  imu_state_i_.SetOrientation(orientation);
  imu_state_i_.SetPosition(position);
  imu_state_i_.SetVelocity(velocity);
  imu_state_i_.SetBiasAcceleration(ba_);
  imu_state_i_.SetBiasGyroscope(bg_);
}

void ImuPreintegration::Increment(double dt, const ImuData& data,
                                  bool compute_jacobian,
                                  bool compute_covariance) {
  // should throw a run time error here based on bad timing

  Eigen::Vector3d w = data.angular_velocity - bg_;
  Eigen::Vector3d a = data.linear_acceleration - ba_;

  if (compute_covariance) {
    Eigen::Matrix<double, 9, 9> A;
    A.setIdentity();
    A.block<3, 3>(ES_Q, ES_Q) = beam_common::expmap(w * dt).conjugate().matrix();
    A.block<3, 3>(ES_V, ES_Q) = -dt * delta.q.matrix() * beam_common::hat(a);
    A.block<3, 3>(ES_P, ES_Q) = -0.5 * dt * dt * delta.q.matrix() * beam_common::hat(a);
    A.block<3, 3>(ES_P, ES_V) = dt * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 9, 6> B;
    B.setZero();
    B.block<3, 3>(ES_Q, ES_BG - ES_BG) = dt * beam_common::right_Jacobian(w * dt);
    B.block<3, 3>(ES_V, ES_BA - ES_BG) = dt * delta.q.matrix();
    B.block<3, 3>(ES_P, ES_BA - ES_BG) = 0.5 * dt * dt * delta.q.matrix();

    Eigen::Matrix<double, 6, 6> white_noise_cov;
    double inv_dt = 1.0 / std::max(dt, 1.0e-7);
    white_noise_cov.setZero();
    white_noise_cov.block<3, 3>(ES_BG - ES_BG, ES_BG - ES_BG) = params_.cov_w * inv_dt;
    white_noise_cov.block<3, 3>(ES_BA - ES_BG, ES_BA - ES_BG) = params_.cov_a * inv_dt;

    delta.cov.block<9, 9>(ES_Q, ES_Q) = A * delta.cov.block<9, 9>(0, 0) * A.transpose() + B * white_noise_cov * B.transpose();
    delta.cov.block<3, 3>(ES_BG, ES_BG) += params_.cov_bg * dt;
    delta.cov.block<3, 3>(ES_BA, ES_BA) += params_.cov_ba * dt;
  }

  if (compute_jacobian) {
    jacobian.dp_dbg += dt * jacobian.dv_dbg - 0.5 * dt * dt * delta.q.matrix() * beam_common::hat(a) * jacobian.dq_dbg;
    jacobian.dp_dba += dt * jacobian.dv_dba - 0.5 * dt * dt * delta.q.matrix();
    jacobian.dv_dbg -= dt * delta.q.matrix() * beam_common::hat(a) * jacobian.dq_dbg;
    jacobian.dv_dba -= dt * delta.q.matrix();
    jacobian.dq_dbg = beam_common::expmap(w * dt).conjugate().matrix() * jacobian.dq_dbg - dt * beam_common::right_Jacobian(w * dt);
  }

  delta.t = delta.t + dt;
  delta.p = delta.p + dt * delta.v + 0.5 * dt * dt * (delta.q * a);
  delta.v = delta.v + dt * (delta.q * a);
  delta.q = (delta.q * beam_common::expmap(w * dt)).normalized();
}

bool ImuPreintegration::Integrate(double t, bool compute_jacobian,
                                  bool compute_covariance) {
  if (imu_data_buffer_.size() == 0) return false;
  ResetMotionEstimate();

  for (size_t i = 0; i + 1 < imu_data_buffer_.size(); ++i) {
    const ImuData& d = imu_data_buffer_[i];
    Increment(imu_data_buffer_[i + 1].time.toSec() - d.time.toSec(), d,
              compute_jacobian, compute_covariance);
  }

  // make some assertions here about the timing of data in the buffer

  Increment(t - imu_data_buffer_.back().time.toSec(), imu_data_buffer_.back(),
            compute_jacobian, compute_covariance);

  return true;
}

ImuState ImuPreintegration::PredictState(ImuState& imu_state) {
  fuse_core::Matrix3d R_i = imu_state.OrientationQuat().toRotationMatrix();
  fuse_core::Matrix3d R_j = R_i * delta.q.matrix();
  fuse_core::Vector3d V_j = imu_state.VelocityVec() +
                            gravitational_acceleration_ * delta.t +
                            R_i * delta.v;
  fuse_core::Vector3d P_j =
      imu_state.PositionVec() + imu_state.VelocityVec() * delta.t +
      0.5 * gravitational_acceleration_ * delta.t * delta.t + R_i * delta.p;

  Eigen::Quaterniond R_j_quat(R_j);

  ros::Time t_new = imu_state.Stamp() + ros::Duration(delta.t);

  ImuState new_imu_state(t_new);
  new_imu_state.SetOrientation(R_j_quat);
  new_imu_state.SetVelocity(V_j);
  new_imu_state.SetPosition(P_j);
  new_imu_state.SetBiasAcceleration(imu_state.BiasAccelerationVec());
  new_imu_state.SetBiasGyroscope(imu_state.BiasGyroscopeVec());

  return new_imu_state;
}

beam_constraints::frame_to_frame::ImuState3DStampedTransaction
ImuPreintegration::RegisterNewImuPreintegrationFactor() {
  // determine first time, last time, and duration of window
  // ros::Time t_i = imu_data_buffer_.front().time;
  // ros::Time t_j = imu_data_buffer_.back().time;
  // double del_t_ij = ros::Duration(t_j - t_i).toSec();

  // // build transaction
  // beam_constraints::frame_to_frame::ImuState3DStampedTransaction transaction(
  //     t_j);

  // // set first imu state at origin
  // if (first_window_) {
  //   imu_state_i_.InstantiateFuseVariables(t_i);
  //   imu_state_i_.SetBiasAcceleration(init_ba_);
  //   imu_state_i_.SetBiasGyroscope(init_bg_);

  //   Eigen::Quaterniond R_i_quat_init(Eigen::Quaterniond::FromTwoVectors(
  //       imu_data_buffer_.front().linear_acceleration,
  //       Eigen::Vector3d::UnitZ()));

  //   imu_state_i_.SetOrientation(R_i_quat_init);

  //   Eigen::Matrix<double, 15, 15> imu_state_prior_covariance;
  //   imu_state_prior_covariance.setIdentity();
  //   imu_state_prior_covariance *= imu_state_prior_noise_;

  //   transaction.AddImuStatePrior(
  //       imu_state_i_.Orientation(), imu_state_i_.Velocity(),
  //       imu_state_i_.Position(), imu_state_i_.BiasAcceleration(),
  //       imu_state_i_.BiasGyroscope(), imu_state_prior_covariance,
  //       params_.source);

  //   first_window_ = false;
  // }

  // // estimate motion and covariance over window
  // fuse_core::Matrix3d delta_R_ij;
  // fuse_core::Vector3d delta_V_ij;
  // fuse_core::Vector3d delta_P_ij;
  // fuse_core::Matrix9d Covariance_ij;
  // Integrate(delta_R_ij, delta_V_ij, delta_P_ij, Covariance_ij);

  // ImuState imu_state_j(t_j);
  // imu_state_j.SetBiasAcceleration(imu_state_i_.BiasAccelerationVec());
  // imu_state_j.SetBiasGyroscope(imu_state_i_.BiasGyroscopeVec());
  // PredictState(imu_state_j, del_t_ij, delta_R_ij, delta_V_ij, delta_P_ij);
}

}}  // namespace beam_models::frame_to_frame
