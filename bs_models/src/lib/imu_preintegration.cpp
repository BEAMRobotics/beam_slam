#include <bs_models/imu_preintegration.h>

#include <fuse_core/transaction.h>

#include <beam_utils/math.h>

namespace bs_models {

ImuPreintegration::ImuPreintegration(const Params &params) : params_(params) {
  CheckParameters();
  SetPreintegrator();
}

ImuPreintegration::ImuPreintegration(const Params &params,
                                     const Eigen::Vector3d &init_bg,
                                     const Eigen::Vector3d &init_ba)
    : params_(params), bg_(init_bg), ba_(init_ba) {
  CheckParameters();
  SetPreintegrator();
}

void ImuPreintegration::AddToBuffer(const sensor_msgs::Imu &msg) {
  bs_common::IMUData imu_data(msg);
  AddToBuffer(imu_data);
}

void ImuPreintegration::AddToBuffer(const bs_common::IMUData &imu_data) {
  pre_integrator_ij.data.push_back(imu_data);
  pre_integrator_kj.data.push_back(imu_data);
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
  pre_integrator_kj.cov_w = params_.cov_gyro_noise;
  pre_integrator_kj.cov_a = params_.cov_accel_noise;
  pre_integrator_kj.cov_bg = params_.cov_gyro_bias;
  pre_integrator_kj.cov_ba = params_.cov_accel_bias;
}

void ImuPreintegration::ResetPreintegrator(const ros::Time &t_now) {
  pre_integrator_ij.Reset();
  pre_integrator_kj.data.erase(
      std::remove_if(pre_integrator_kj.data.begin(),
                     pre_integrator_kj.data.end(),
                     [&](const bs_common::IMUData &d) { return d.t < t_now; }),
      pre_integrator_kj.data.end());
  pre_integrator_kj.Reset();
  pre_integrator_kj.data.erase(
      std::remove_if(pre_integrator_kj.data.begin(),
                     pre_integrator_kj.data.end(),
                     [&](const bs_common::IMUData &d) { return d.t < t_now; }),
      pre_integrator_kj.data.end());
}

void ImuPreintegration::SetStart(
    const ros::Time &t_start,
    fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU,
    fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU,
    fuse_variables::VelocityLinear3DStamped::SharedPtr velocity) {

  // remove data in buffer that is before the start state
  pre_integrator_ij.data.erase(std::remove_if(pre_integrator_ij.data.begin(),
                                              pre_integrator_ij.data.end(),
                                              [&](const bs_common::IMUData &d) {
                                                return d.t < t_start;
                                              }),
                               pre_integrator_ij.data.end());
  pre_integrator_kj.data.erase(std::remove_if(pre_integrator_kj.data.begin(),
                                              pre_integrator_kj.data.end(),
                                              [&](const bs_common::IMUData &d) {
                                                return d.t < t_start;
                                              }),
                               pre_integrator_kj.data.end());

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

  imu_state_i_ = imu_state_i;

  // copy start IMU state to initialize kth frame between keyframes
  imu_state_k_ = imu_state_i_;
}

bs_common::ImuState
ImuPreintegration::PredictState(const bs_common::PreIntegrator &pre_integrator,
                                const bs_common::ImuState &imu_state_curr,
                                const ros::Time &t_now) {
  // get commonly used variables
  const double &dt = pre_integrator.delta.t.toSec();
  const Eigen::Matrix3d &q_curr = imu_state_curr.OrientationMat();

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

bool ImuPreintegration::GetPose(Eigen::Matrix4d &T_WORLD_IMU,
                                const ros::Time &t_now) {
  // check requested time
  if (t_now < pre_integrator_kj.data.front().t ||
      pre_integrator_kj.data.empty()) {
    ROS_WARN("Requested time is outside of window, or no imu data available.");
    return false;
  }

  if (!pre_integrator_kj.data.empty()) {
    // integrate between frames
    pre_integrator_kj.Integrate(t_now, imu_state_i_.GyroBiasVec(),
                                imu_state_i_.AccelBiasVec(), false, false);
  }

  // predict state at end of window using integrated IMU measurements
  imu_state_k_ = PredictState(pre_integrator_kj, imu_state_k_, t_now);

  // populate transformation matrix
  beam::QuaternionAndTranslationToTransformMatrix(
      imu_state_k_.OrientationQuat(), imu_state_k_.PositionVec(), T_WORLD_IMU);

  // remove data in buffer that is before the new state k
  pre_integrator_kj.data.erase(
      std::remove_if(pre_integrator_kj.data.begin(),
                     pre_integrator_kj.data.end(),
                     [&](const bs_common::IMUData &d) { return d.t < t_now; }),
      pre_integrator_kj.data.end());

  return true;
}

fuse_core::Transaction::SharedPtr
ImuPreintegration::RegisterNewImuPreintegratedFactor(
    const ros::Time &t_now,
    fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU,
    fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU,
    bool update_velocity) {
  bs_constraints::relative_pose::ImuState3DStampedTransaction transaction(
      t_now);
  // check requested time
  if (t_now < pre_integrator_ij.data.front().t ||
      pre_integrator_ij.data.empty()) {
    ROS_WARN(
        "Requested time is outside of window, or no imu data is available.");
    return nullptr;
  }

  // generate prior constraint at start
  if (first_window_) {
    Eigen::Matrix<double, 15, 15> prior_covariance{
        Eigen::Matrix<double, 15, 15>::Identity()};
    for (int i = 0; i < 15; i++) {
      prior_covariance(i, i) = params_.cov_prior_noise;
    }

    // Add relative constraints and variables for first key frame
    transaction.AddPriorImuStateConstraint(imu_state_i_, prior_covariance,
                                           "FIRST_IMU_STATE_PRIOR");
    transaction.AddImuStateVariables(imu_state_i_);

    first_window_ = false;
  }

  // integrate between key frames, incrementally calculating covariance and
  // jacobians
  pre_integrator_ij.Integrate(t_now, imu_state_i_.GyroBiasVec(),
                              imu_state_i_.AccelBiasVec(), true, true);

  // predict state at end of window using integrated imu measurements
  bs_common::ImuState imu_state_j =
      PredictState(pre_integrator_ij, imu_state_i_, t_now);

  // Add relative constraints and variables between key frames
  transaction.AddRelativeImuStateConstraint(imu_state_i_, imu_state_j,
                                            pre_integrator_ij);
  transaction.AddImuStateVariables(imu_state_j);

  // update orientation and position of predicted imu state with arguments
  if (R_WORLD_IMU && t_WORLD_IMU) {
    imu_state_j.SetOrientation(R_WORLD_IMU->data());
    imu_state_j.SetPosition(t_WORLD_IMU->data());
    if (update_velocity) {
      Eigen::Vector3d new_velocity =
          (imu_state_j.PositionVec() - imu_state_i_.PositionVec()) /
          (t_now.toSec() - imu_state_i_.Stamp().toSec());
      imu_state_j.SetVelocity(new_velocity);
    }
  }

  // move predicted state to previous state
  imu_state_i_ = imu_state_j;

  // copy state i to kth frame
  imu_state_k_ = imu_state_i_;

  ResetPreintegrator(t_now);

  return transaction.GetTransaction();
}

void ImuPreintegration::UpdateGraph(fuse_core::Graph::SharedPtr graph_msg) {
  // if update was successful then also reset buffer and state k
  if (imu_state_i_.Update(graph_msg)) {
    // reset kj integrator and add all the current windows messages to it
    pre_integrator_kj.data.clear();
    for (auto &d : pre_integrator_ij.data) {
      pre_integrator_kj.data.push_back(d);
    }
    // reset state k to state i
    imu_state_k_ = imu_state_i_;
  }
}

} // namespace bs_models
