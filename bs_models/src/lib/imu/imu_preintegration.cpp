#include <bs_models/imu/imu_preintegration.h>

#include <fuse_core/transaction.h>
#include <geometry_msgs/PoseStamped.h>
#include <nlohmann/json.hpp>

#include <beam_utils/filesystem.h>
#include <beam_utils/math.h>

namespace bs_models {

bool ImuPreintegration::Params::LoadFromJSON(const std::string& path) {
  nlohmann::json J;
  beam::ReadJson(path, J);
  if (!J.contains("cov_gyro_noise")) {
    BEAM_ERROR("Missing or misspelt parameter: 'cov_gryo_noise'");
    return false;
  }
  if (!J.contains("cov_accel_noise")) {
    BEAM_ERROR("Missing or misspelt parameter: 'cov_accel_noise'");
    return false;
  }
  if (!J.contains("cov_gyro_bias")) {
    BEAM_ERROR("Missing or misspelt parameter: 'cov_gyro_bias'");
    return false;
  }
  if (!J.contains("cov_accel_bias")) {
    BEAM_ERROR("Missing or misspelt parameter: 'cov_accel_bias'");
    return false;
  }

  cov_gyro_noise = Eigen::Matrix3d::Identity() * J["cov_gyro_noise"];
  cov_accel_noise = Eigen::Matrix3d::Identity() * J["cov_accel_noise"];
  cov_gyro_bias = Eigen::Matrix3d::Identity() * J["cov_gyro_bias"];
  cov_accel_bias = Eigen::Matrix3d::Identity() * J["cov_accel_bias"];

  return true;
}

ImuPreintegration::ImuPreintegration(const std::string& source,
                                     const Params& params,
                                     const double info_weight,
                                     bool add_prior_on_first_window)
    : source_(source),
      params_(params),
      info_weight_(info_weight),
      add_prior_on_first_window_(add_prior_on_first_window) {
  CheckParameters();
  SetPreintegrator();
}

ImuPreintegration::ImuPreintegration(const std::string& source,
                                     const Params& params,
                                     const Eigen::Vector3d& init_bg,
                                     const Eigen::Vector3d& init_ba,
                                     const double info_weight,
                                     bool add_prior_on_first_window)
    : source_(source),
      params_(params),
      bg_(init_bg),
      ba_(init_ba),
      info_weight_(info_weight),
      add_prior_on_first_window_(add_prior_on_first_window) {
  CheckParameters();
  SetPreintegrator();
}

void ImuPreintegration::CheckParameters() {
  if (params_.cov_prior_noise <= 0.0) {
    ROS_ERROR("Prior noise on IMU state must be greater than zero, value: %.9f",
              params_.cov_prior_noise);
    throw std::invalid_argument{"Inputs to ImuPreintegration invalid."};
  }
}

void ImuPreintegration::SetPreintegrator() {
  pre_integrator_ij_.cov_w = params_.cov_gyro_noise;
  pre_integrator_ij_.cov_a = params_.cov_accel_noise;
  pre_integrator_ij_.cov_bg = params_.cov_gyro_bias;
  pre_integrator_ij_.cov_ba = params_.cov_accel_bias;
  pre_integrator_kj_ = pre_integrator_ij_;
}

void ImuPreintegration::AddToBuffer(const sensor_msgs::Imu& msg) {
  bs_common::IMUData imu_data(msg);
  AddToBuffer(imu_data);
}

void ImuPreintegration::AddToBuffer(const bs_common::IMUData& imu_data) {
  std::unique_lock<std::mutex> lk(preint_mutex_);
  pre_integrator_ij_.data.emplace(imu_data.t, imu_data);
  pre_integrator_kj_.data.emplace(imu_data.t, imu_data);
}

PoseWithCovariance ImuPreintegration::GetPose(const ros::Time& t_now) {
  if (t_now < imu_state_k_.Stamp()) {
    throw std::runtime_error{
        "Requested time is before current imu state. Request a "
        "pose at a timestamp >= the most recent imu measurement"};
  }
  std::unique_lock<std::mutex> lk(preint_mutex_);
  // integrate between frames if there is data to integrate
  if (!pre_integrator_kj_.data.empty()) {
    pre_integrator_kj_.Integrate(t_now, imu_state_i_.GyroBiasVec(),
                                 imu_state_i_.AccelBiasVec(), false, true,
                                 false);
  }

  // predict state at end of window using integrated IMU measurements
  imu_state_k_ = PredictState(pre_integrator_kj_, imu_state_k_, t_now);

  // populate transformation matrix
  Eigen::Matrix4d T_WORLD_IMU;
  beam::QuaternionAndTranslationToTransformMatrix(
      imu_state_k_.OrientationQuat(), imu_state_k_.PositionVec(), T_WORLD_IMU);

  // remove data in buffer that is before the new state k
  pre_integrator_kj_.Clear(t_now);
  pre_integrator_kj_.Reset();

  // extract the computed covariance if requested
  Eigen::Matrix<double, 6, 6> covariance =
      pre_integrator_kj_.delta.cov.block<6, 6>(0, 0);
  return std::make_pair(T_WORLD_IMU, covariance);
}

PoseWithCovariance ImuPreintegration::GetRelativeMotion(
    const ros::Time& t1, const ros::Time& t2, Eigen::Vector3d& velocity_t2) {
  if (pre_integrator_ij_.data.empty()) {
    throw std::runtime_error{
        "No data in preintegrator, cannot retrieve relative motion."};
  } else if (t1 < imu_state_i_.Stamp()) {
    throw std::runtime_error{
        "Requested time is before current window of measurements."};
  } else if (t1 > pre_integrator_ij_.data.rbegin()->first) {
    throw std::runtime_error{
        "Requested start time is after the end of the current window of imu "
        "measurements."};
  } else if (t2 < pre_integrator_ij_.data.begin()->first) {
    throw std::runtime_error{
        "Requested end time is before the start of the current window of imu "
        "measurements."};
  } else if (t1 >= t2) {
    throw std::runtime_error{"Start time of window must precede end times."};
  }
  std::unique_lock<std::mutex> lk(preint_mutex_);
  // get state at t1
  bs_common::ImuState imu_state_1;
  if (window_states_.find(t1.toNSec()) == window_states_.end()) {
    pre_integrator_ij_.Integrate(t1, imu_state_i_.GyroBiasVec(),
                                 imu_state_i_.AccelBiasVec(), false, false,
                                 false);
    imu_state_1 = PredictState(pre_integrator_ij_, imu_state_i_, t1);
  } else {
    imu_state_1 = window_states_[t1.toNSec()];
  }

  // copy preintegrator ij
  bs_common::PreIntegrator pre_integrator = pre_integrator_ij_;
  pre_integrator.Clear(t1);
  pre_integrator.Reset();

  // integrate to t2
  pre_integrator.Integrate(t2, imu_state_i_.GyroBiasVec(),
                           imu_state_i_.AccelBiasVec(), false, true, false);
  // get state at t2
  bs_common::ImuState imu_state_2 =
      PredictState(pre_integrator, imu_state_1, t2);

  // store for potential next t1
  window_states_.emplace(t2.toNSec(), imu_state_2);

  // get poses at each state
  Eigen::Matrix4d T_WORLD_IMUSTATE2;
  beam::QuaternionAndTranslationToTransformMatrix(imu_state_2.OrientationQuat(),
                                                  imu_state_2.PositionVec(),
                                                  T_WORLD_IMUSTATE2);
  Eigen::Matrix4d T_WORLD_IMUSTATE1;
  beam::QuaternionAndTranslationToTransformMatrix(imu_state_1.OrientationQuat(),
                                                  imu_state_1.PositionVec(),
                                                  T_WORLD_IMUSTATE1);

  // compute relative motion between them
  Eigen::Matrix4d T_IMUSTATE1_IMUSTATE2 =
      beam::InvertTransform(T_WORLD_IMUSTATE1) * T_WORLD_IMUSTATE2;

  // get covariance
  Eigen::Matrix<double, 6, 6> covariance =
      pre_integrator.delta.cov.block<6, 6>(0, 0);

  // get velocity
  velocity_t2 = imu_state_2.VelocityVec();
  return std::make_pair(T_IMUSTATE1_IMUSTATE2, covariance);
}

void ImuPreintegration::SetStart(
    const ros::Time& t_start,
    fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU,
    fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU,
    fuse_variables::VelocityLinear3DStamped::SharedPtr velocity) {
  std::unique_lock<std::mutex> lk(preint_mutex_);
  // remove data in buffer that is before the start state
  pre_integrator_ij_.Clear(t_start);
  pre_integrator_ij_.Reset();
  pre_integrator_kj_.Clear(t_start);
  pre_integrator_kj_.Reset();

  // set IMU state
  bs_common::ImuState imu_state_i(t_start);
  if (R_WORLD_IMU) { imu_state_i.SetOrientation(R_WORLD_IMU->data()); }
  if (t_WORLD_IMU) { imu_state_i.SetPosition(t_WORLD_IMU->data()); }
  if (velocity) { imu_state_i.SetVelocity(velocity->data()); }
  imu_state_i.SetGyroBias(bg_);
  imu_state_i.SetAccelBias(ba_);

  imu_state_i_ = imu_state_i;
  imu_state_k_ = imu_state_i;
}

bs_common::ImuState ImuPreintegration::PredictState(
    const bs_common::PreIntegrator& pre_integrator,
    const bs_common::ImuState& imu_state_curr, const ros::Time& t_now) {
  // get commonly used variables
  const double& dt = pre_integrator.delta.t.toSec();
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
  if (t_now != ros::Time(0)) { t_new = t_now; }

  // return predicted IMU state
  bs_common::ImuState imu_state_new(t_new, q_new, p_new, v_new,
                                    imu_state_curr.GyroBiasVec(),
                                    imu_state_curr.AccelBiasVec());
  return imu_state_new;
}

fuse_core::Transaction::SharedPtr
    ImuPreintegration::RegisterNewImuPreintegratedFactor(
        const ros::Time& t_now,
        fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU,
        fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU,
        fuse_variables::VelocityLinear3DStamped::SharedPtr velocity) {
  bs_constraints::ImuState3DStampedTransaction transaction(t_now);
  std::unique_lock<std::mutex> lk(preint_mutex_);
  // check requested time
  if (pre_integrator_ij_.data.empty()) {
    ROS_WARN("Cannot register IMU factor, no imu data is available.");
    return nullptr;
  }
  if (t_now < pre_integrator_ij_.data.begin()->first) {
    ROS_WARN(
        "Cannot register IMU factor, requested time is prior to the front "
        "of the window. Request a pose at a timestamp >= the previous call.");
    return nullptr;
  }

  // generate prior constraint at start
  if (first_window_ && add_prior_on_first_window_) {
    Eigen::Matrix<double, 15, 15> prior_covariance =
        params_.cov_prior_noise * Eigen::Matrix<double, 15, 15>::Identity();

    // Add relative constraints and variables for first key frame
    transaction.AddPriorImuStateConstraint(imu_state_i_, prior_covariance,
                                           source_);
    transaction.AddImuStateVariables(imu_state_i_);
    first_window_ = false;
  }

  // if current time is equal to the first imu time, then we can't add a
  // relative constraint
  if (t_now == pre_integrator_ij_.data.begin()->first) {
    return transaction.GetTransaction();
  }

  // integrate between key frames, incrementally calculating covariance and
  // jacobians
  pre_integrator_ij_.Integrate(t_now, imu_state_i_.GyroBiasVec(),
                               imu_state_i_.AccelBiasVec(), true, true, true);

  // predict state at end of window using integrated imu measurements
  bs_common::ImuState imu_state_j =
      PredictState(pre_integrator_ij_, imu_state_i_, t_now);

  // Add relative constraints and variables between key frames
  transaction.AddRelativeImuStateConstraint(
      imu_state_i_, imu_state_j, pre_integrator_ij_, info_weight_, source_);
  transaction.AddImuStateVariables(imu_state_j);

  // update orientation, position and velocity of predicted imu state with
  // arguments
  if (R_WORLD_IMU && t_WORLD_IMU && velocity) {
    // update with new estimate
    imu_state_j.SetOrientation(R_WORLD_IMU->data());
    imu_state_j.SetPosition(t_WORLD_IMU->data());
    imu_state_j.SetVelocity(velocity->data());
  }

  // move predicted state to previous state
  imu_state_i_ = imu_state_j;
  imu_state_k_ = imu_state_j;

  // clear and reset preintegrator
  pre_integrator_ij_.Clear(t_now);
  pre_integrator_ij_.Reset();
  pre_integrator_kj_.Clear(t_now);
  pre_integrator_kj_.Reset();

  // clear state storage within the window
  window_states_.clear();
  return transaction.GetTransaction();
}

void ImuPreintegration::UpdateGraph(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  std::unique_lock<std::mutex> lk(preint_mutex_);
  // update state i with all info, reset state k to updated state i
  if (imu_state_i_.Update(graph_msg)) {
    // reset kj integrator to the ij integrator
    pre_integrator_kj_ = pre_integrator_ij_;
    // reset state k to state i
    imu_state_k_ = imu_state_i_;
    bg_ = imu_state_i_.GyroBiasVec();
    ba_ = imu_state_i_.AccelBiasVec();
  }
  // clear state storage within the window
  window_states_.clear();
}

void ImuPreintegration::UpdateState(
    const fuse_variables::Position3DStamped position,
    const fuse_variables::Orientation3DStamped orientation,
    const fuse_variables::VelocityLinear3DStamped velocity,
    const bs_variables::GyroscopeBias3DStamped gyro_bias,
    const bs_variables::AccelerationBias3DStamped accel_bias) {
  imu_state_i_.SetStamp(position.stamp());
  imu_state_i_.SetPosition(position);
  imu_state_i_.SetOrientation(orientation);
  imu_state_i_.SetVelocity(velocity);
  imu_state_i_.SetGyroBias(gyro_bias);
  imu_state_i_.SetAccelBias(accel_bias);
}

void ImuPreintegration::Clear() {
  std::unique_lock<std::mutex> lk(preint_mutex_);
  pre_integrator_kj_.data.clear();
  pre_integrator_kj_.Reset();
  pre_integrator_ij_.data.clear();
  pre_integrator_ij_.Reset();
}

std::string ImuPreintegration::PrintBuffer() {
  std::string str;
  for (const auto& [t, _] : pre_integrator_ij_.data) {
    str += "IMU time: " + std::to_string(t.toSec()) + "\n";
  }
  return str;
}

size_t ImuPreintegration::CurrentBufferSize() {
  return pre_integrator_ij_.data.size();
}

void ImuPreintegration::Reset() {
  Clear();
  window_states_.clear();
  first_window_ = true;
}

} // namespace bs_models
