#include <bs_models/imu/imu_preintegration.h>
#include <geometry_msgs/PoseStamped.h>

#include <fuse_core/transaction.h>

#include <beam_utils/math.h>
#include <nlohmann/json.hpp>

namespace bs_models {

bool ImuPreintegration::Params::LoadFromJSON(const std::string& path) {
  nlohmann::json J;
  beam::ReadJson(path, J);
  try {
    cov_gyro_noise = Eigen::Matrix3d::Identity() * J["cov_gyro_noise"];
  } catch (...) {
    BEAM_ERROR("Missing or misspelt parameter: 'cov_gryo_noise'");
    return false;
  }
  try {
    cov_accel_noise = Eigen::Matrix3d::Identity() * J["cov_accel_noise"];
  } catch (...) {
    BEAM_ERROR("Missing or misspelt parameter: 'cov_accel_noise'");
    return false;
  }
  try {
    cov_gyro_bias = Eigen::Matrix3d::Identity() * J["cov_gyro_bias"];
  } catch (...) {
    BEAM_ERROR("Missing or misspelt parameter: 'cov_gyro_bias'");
    return false;
  }
  try {
    cov_accel_bias = Eigen::Matrix3d::Identity() * J["cov_accel_bias"];
  } catch (...) {
    BEAM_ERROR("Missing or misspelt parameter: 'cov_accel_bias'");
    return false;
  }
  return true;
}

ImuPreintegration::ImuPreintegration(const Params& params) : params_(params) {
  CheckParameters();
  SetPreintegrator();
}

ImuPreintegration::ImuPreintegration(const Params& params, const Eigen::Vector3d& init_bg,
                                     const Eigen::Vector3d& init_ba)
    : params_(params), bg_(init_bg), ba_(init_ba) {
  CheckParameters();
  SetPreintegrator();
}

void ImuPreintegration::CheckParameters() {
  if (params_.cov_prior_noise <= 0) {
    BEAM_ERROR("Prior noise on IMU state must be positive");
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
  pre_integrator_ij_.data.push_back(imu_data);
  pre_integrator_kj_.data.push_back(imu_data);
}

beam::opt<PoseWithCovariance> ImuPreintegration::GetPose(const ros::Time& t_now) {
  // check requested time given an empty integrator
  if (t_now < imu_state_k_.Stamp()) {
    ROS_WARN_STREAM(__func__ << ": Requested time is before current imu state. Request a pose "
                                "at a timestamp >= the most recent imu measurement.");
    return {};
  }

  // integrate between frames if there is data to integrate
  if (!pre_integrator_kj_.data.empty()) {
    pre_integrator_kj_.Integrate(t_now, imu_state_i_.GyroBiasVec(), imu_state_i_.AccelBiasVec(),
                                 false, true);
  }

  // predict state at end of window using integrated IMU measurements
  imu_state_k_ = PredictState(pre_integrator_kj_, imu_state_k_, t_now);

  // populate transformation matrix
  Eigen::Matrix4d T_WORLD_IMU;
  beam::QuaternionAndTranslationToTransformMatrix(imu_state_k_.OrientationQuat(),
                                                  imu_state_k_.PositionVec(), T_WORLD_IMU);

  // remove data in buffer that is before the new state k
  pre_integrator_kj_.Clear(t_now);
  pre_integrator_kj_.Reset();

  // extract the computed covariance if requested
  Eigen::Matrix<double, 6, 6> covariance = pre_integrator_kj_.delta.cov.block<6, 6>(0, 0);

  return std::make_pair(T_WORLD_IMU, covariance);
}

beam::opt<PoseWithCovariance> ImuPreintegration::GetRelativeMotion(const ros::Time& t1,
                                                                   const ros::Time& t2) {
  if (t1 < pre_integrator_ij_.data.front().t || t1 > pre_integrator_ij_.data.back().t) {
    ROS_WARN_STREAM(
        __func__ << ": Requested start time is outside the current window of imu measurements.");
    return {};
  }
  if (t2 < pre_integrator_ij_.data.front().t) {
    ROS_WARN_STREAM(
        __func__
        << ": Requested end time is before the start of the current window of imu measurements.");
    return {};
  }

  // copy preintegrator ij and remove messages before t1
  bs_common::PreIntegrator pre_integrator = pre_integrator_ij_;
  pre_integrator.Clear(t1);
  pre_integrator.Reset();

  // set initial state to identity
  bs_common::ImuState imu_state_1(t1);
  imu_state_1.SetPosition(Eigen::Vector3d::Zero());
  imu_state_1.SetOrientation(Eigen::Quaterniond(Eigen::Matrix3d::Identity()));

  // integrate to t2 using recent bias estimate
  pre_integrator.Integrate(t2, imu_state_i_.GyroBiasVec(), imu_state_i_.AccelBiasVec(), false,
                           true);

  // predict state
  bs_common::ImuState imu_state_2 = PredictState(pre_integrator, imu_state_1, t2);

  // populate transformation matrix and return
  Eigen::Matrix4d T_IMUSTATE1_IMUSTATE2;
  beam::QuaternionAndTranslationToTransformMatrix(imu_state_2.OrientationQuat(),
                                                  imu_state_2.PositionVec(), T_IMUSTATE1_IMUSTATE2);
  // get covariance
  Eigen::Matrix<double, 6, 6> covariance = pre_integrator.delta.cov.block<6, 6>(0, 0);

  return std::make_pair(T_IMUSTATE1_IMUSTATE2, covariance);
}

void ImuPreintegration::SetStart(const ros::Time& t_start,
                                 fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU,
                                 fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU,
                                 fuse_variables::VelocityLinear3DStamped::SharedPtr velocity) {
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

bs_common::ImuState ImuPreintegration::PredictState(const bs_common::PreIntegrator& pre_integrator,
                                                    const bs_common::ImuState& imu_state_curr,
                                                    const ros::Time& t_now) {
  // get commonly used variables
  const double& dt = pre_integrator.delta.t.toSec();
  const Eigen::Matrix3d& q_curr = imu_state_curr.OrientationMat();

  // predict new states
  Eigen::Quaterniond q_new(q_curr * pre_integrator.delta.q.matrix());
  // TODO: do we subtract gravity now? (it is 0, 0, 9.8)
  Eigen::Vector3d v_new =
      imu_state_curr.VelocityVec() + GRAVITY_WORLD * dt + q_curr * pre_integrator.delta.v;
  Eigen::Vector3d p_new = imu_state_curr.PositionVec() + imu_state_curr.VelocityVec() * dt +
                          0.5 * GRAVITY_WORLD * dt * dt + q_curr * pre_integrator.delta.p;

  // set time
  ros::Time t_new = imu_state_curr.Stamp() + pre_integrator.delta.t;
  if (t_now != ros::Time(0)) { t_new = t_now; }

  // return predicted IMU state
  bs_common::ImuState imu_state_new(t_new, q_new, p_new, v_new, imu_state_curr.GyroBiasVec(),
                                    imu_state_curr.AccelBiasVec());
  return imu_state_new;
}

fuse_core::Transaction::SharedPtr ImuPreintegration::RegisterNewImuPreintegratedFactor(
    const ros::Time& t_now, fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU,
    fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU,
    fuse_variables::VelocityLinear3DStamped::SharedPtr velocity) {
  bs_constraints::relative_pose::ImuState3DStampedTransaction transaction(t_now);

  // check requested time
  if (pre_integrator_ij_.data.empty()) {
    ROS_WARN("Cannot register IMU factor, no imu data is available.");
    return nullptr;
  }
  if (t_now < pre_integrator_ij_.data.front().t) {
    ROS_WARN("Cannot register IMU factor, requested time is prior to the front "
             "of the window. Request a pose at a timestamp >= the previous call.");
    return nullptr;
  }

  // generate prior constraint at start
  if (first_window_) {
    Eigen::Matrix<double, 15, 15> prior_covariance =
        params_.cov_prior_noise * Eigen::Matrix<double, 15, 15>::Identity();

    // Add relative constraints and variables for first key frame
    transaction.AddPriorImuStateConstraint(imu_state_i_, prior_covariance, "FIRST_IMU_STATE_PRIOR");
    transaction.AddImuStateVariables(imu_state_i_);
    first_window_ = false;
  }

  // integrate between key frames, incrementally calculating covariance and
  // jacobians
  pre_integrator_ij_.Integrate(t_now, imu_state_i_.GyroBiasVec(), imu_state_i_.AccelBiasVec(), true,
                               true);

  // predict state at end of window using integrated imu measurements
  bs_common::ImuState imu_state_j = PredictState(pre_integrator_ij_, imu_state_i_, t_now);

  // Add relative constraints and variables between key frames
  transaction.AddRelativeImuStateConstraint(imu_state_i_, imu_state_j, pre_integrator_ij_);
  transaction.AddImuStateVariables(imu_state_j);

  // update orientation, position and velocity of predicted imu state with arguments
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
  return transaction.GetTransaction();
}

void ImuPreintegration::UpdateGraph(fuse_core::Graph::ConstSharedPtr graph_msg) {
  // update state i with all info, reset state k to updated state i
  if (imu_state_i_.Update(graph_msg)) {
    // reset kj integrator to the ij integrator
    pre_integrator_kj_ = pre_integrator_ij_;
    // reset state k to state i
    imu_state_k_ = imu_state_i_;
  }
}

void ImuPreintegration::Clear() {
  pre_integrator_kj_.data.clear();
  pre_integrator_kj_.Reset();
  pre_integrator_ij_.data.clear();
  pre_integrator_ij_.Reset();
}

} // namespace bs_models
