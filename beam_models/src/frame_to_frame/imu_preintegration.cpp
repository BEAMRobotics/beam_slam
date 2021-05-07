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
  ImuData imu_data;
  imu_data.t_ros = msg->header.stamp;
  imu_data.t = msg->header.stamp.toSec();
  imu_data.w[0] = msg->angular_velocity.x;
  imu_data.w[1] = msg->angular_velocity.y;
  imu_data.w[2] = msg->angular_velocity.z;
  imu_data.a[0] = msg->linear_acceleration.x;
  imu_data.a[1] = msg->linear_acceleration.y;
  imu_data.a[2] = msg->linear_acceleration.z;
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
  if (orientation->stamp() != position->stamp() ||
      orientation->stamp() != velocity->stamp() ||
      position->stamp() != velocity->stamp()) {
    // throw error
  }

  if (t_start.toSec() > imu_data_buffer_.front().t) {
    int counter = 0;
    while (t_start.toSec() > imu_data_buffer_.at(counter).t) {
      ++counter;
      if (counter == imu_data_buffer_.size()) {
        // throw error
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
  ImuState new_imu_state(t_new, R_j_quat, V_j, P_j,
                         imu_state.BiasGyroscopeVec(),
                         imu_state.BiasAccelerationVec());
  return new_imu_state;
}

beam_constraints::frame_to_frame::ImuState3DStampedTransaction
ImuPreintegration::GetIMUPreintegrationTransaction(
    const ros::Time& t_now,
    fuse_variables::Orientation3DStamped::SharedPtr orientation,
    fuse_variables::Position3DStamped::SharedPtr position) {
  auto it = std::find_if(imu_data_buffer_.begin(), imu_data_buffer_.end(),
                         [&t_now](const ImuPreintegration::ImuData& data) {
                           return data.t_ros == t_now;
                         });

  if (it == imu_data_buffer_.end()) {
    // throw error
  }

  std::shared_ptr<PreIntegrator> pre_integrator_ij;

  pre_integrator_ij->data.insert(pre_integrator_ij->data.end(),
                                 imu_data_buffer_.begin(), it);

  imu_data_buffer_.erase(imu_data_buffer_.begin(), it);

  //pre_integrator_ij->integrate(t_now.toSec(),imu_state_i_.BiasGyroscopeVec(), imu_state_i_.BiasAccelerationVec(), true, true);


  // pre_integrator_ij->data.insert(pre_integrator_ij->data.end(),
  //                                imu_data_buffer_.begin(), it);

  // imu_data_buffer_.erase(imu_data_buffer_.begin(), it);

  // pre_integrator_ij->integrate(t_now.toSec(),
  // imu_state_i_.BiasGyroscopeVec(), imu_state_i_.BiasAccelerationVec(), true,
  // true);

  //pre_integrator_ij->reset();

  // double t_double = t_now.toSec();
  // pre_integrator_ij->integrate(t_double,
  // bg_, ba_, true,
  // true);


}

// beam_constraints::frame_to_frame::ImuState3DStampedTransaction
// GetIMUPreintegrationTransaction() {
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
// }

}}  // namespace beam_models::frame_to_frame
