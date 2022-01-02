#include <bs_models/imu_preintegration.h>

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

ImuPreintegration::ImuPreintegration(const Params& params,
                                     const Eigen::Vector3d& init_bg,
                                     const Eigen::Vector3d& init_ba)
    : params_(params), bg_(init_bg), ba_(init_ba) {
  CheckParameters();
  SetPreintegrator();
}

void ImuPreintegration::AddToBuffer(const sensor_msgs::Imu& msg) {
  bs_common::IMUData imu_data(msg);
  AddToBuffer(imu_data);
}

void ImuPreintegration::AddToBuffer(const bs_common::IMUData& imu_data) {
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

void ImuPreintegration::ResetPreintegrator(const ros::Time& t_now) {
  pre_integrator_ij.Reset();
  pre_integrator_ij.data.erase(
      std::remove_if(pre_integrator_ij.data.begin(),
                     pre_integrator_ij.data.end(),
                     [&](const bs_common::IMUData& d) { return d.t < t_now; }),
      pre_integrator_ij.data.end());
  pre_integrator_kj.Reset();
  pre_integrator_kj.data.erase(
      std::remove_if(pre_integrator_kj.data.begin(),
                     pre_integrator_kj.data.end(),
                     [&](const bs_common::IMUData& d) { return d.t < t_now; }),
      pre_integrator_kj.data.end());
}

void ImuPreintegration::SetStart(
    const ros::Time& t_start,
    fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU,
    fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU,
    fuse_variables::VelocityLinear3DStamped::SharedPtr velocity) {
  // remove data in buffer that is before the start state
  pre_integrator_ij.data.erase(std::remove_if(pre_integrator_ij.data.begin(),
                                              pre_integrator_ij.data.end(),
                                              [&](const bs_common::IMUData& d) {
                                                return d.t < t_start;
                                              }),
                               pre_integrator_ij.data.end());
  pre_integrator_kj.data.erase(std::remove_if(pre_integrator_kj.data.begin(),
                                              pre_integrator_kj.data.end(),
                                              [&](const bs_common::IMUData& d) {
                                                return d.t < t_start;
                                              }),
                               pre_integrator_kj.data.end());

  // set IMU state
  bs_common::ImuState imu_state_i(t_start);

  if (R_WORLD_IMU) { imu_state_i.SetOrientation(R_WORLD_IMU->data()); }

  if (t_WORLD_IMU) { imu_state_i.SetPosition(t_WORLD_IMU->data()); }

  if (velocity) { imu_state_i.SetVelocity(velocity->data()); }

  imu_state_i.SetGyroBias(bg_);
  imu_state_i.SetAccelBias(ba_);

  imu_state_i_ = imu_state_i;

  // copy start IMU state to initialize kth frame between keyframes
  imu_state_k_ = imu_state_i_;
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

bool ImuPreintegration::GetPose(
    Eigen::Matrix4d& T_WORLD_IMU, const ros::Time& t_now,
    std::shared_ptr<Eigen::Matrix<double, 6, 6>> covariance) {
  // check requested time
  if (t_now < pre_integrator_kj.data.front().t ||
      pre_integrator_kj.data.empty() ||
      t_now < pre_integrator_kj.data.back().t) {
    ROS_WARN("Requested time is outside of window, or no imu data available.");
    return false;
  }

  // integrate between frames if there is data to integrate
  if (!pre_integrator_kj.data.empty()) {
    pre_integrator_kj.Integrate(t_now, imu_state_i_.GyroBiasVec(),
                                imu_state_i_.AccelBiasVec(), true, true);
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
                     [&](const bs_common::IMUData& d) { return d.t < t_now; }),
      pre_integrator_kj.data.end());

  // extract the computed covariance if requested
  if (covariance) {
    *covariance = pre_integrator_kj.delta.cov.block<6, 6>(0, 0);
  }

  return true;
}

fuse_core::Transaction::SharedPtr
    ImuPreintegration::RegisterNewImuPreintegratedFactor(
        const ros::Time& t_now,
        fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU,
        fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU,
        bool update_velocity) {
  bs_constraints::relative_pose::ImuState3DStampedTransaction transaction(
      t_now);
  
  // check requested time
  if (pre_integrator_ij.data.empty()) {
    ROS_WARN("Cannot register IMU factor, no imu data is available.");
    return nullptr;
  }
  if (t_now < pre_integrator_ij.data.front().t ||
      t_now < pre_integrator_ij.data.back().t) {
    ROS_WARN(
        "Cannot register IMU factor, requested time is outside of window.");
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
    pre_integrator_kj.Reset();
    for (auto& d : pre_integrator_ij.data) {
      pre_integrator_kj.data.push_back(d);
    }
    // reset state k to state i
    imu_state_k_ = imu_state_i_;
  }
}

void ImuPreintegration::Clear() {
  pre_integrator_kj.data.clear();
  pre_integrator_kj.Reset();
  pre_integrator_ij.data.clear();
  pre_integrator_ij.Reset();
}

void ImuPreintegration::EstimateParameters(
    const bs_common::InitializedPathMsg& path,
    const std::queue<sensor_msgs::Imu>& imu_buffer,
    const bs_models::ImuPreintegration::Params& params,
    Eigen::Vector3d& gravity, Eigen::Vector3d& bg, Eigen::Vector3d& ba,
    double& scale) {
  // set parameter estimates to 0
  gravity = Eigen::Vector3d::Zero();
  bg = Eigen::Vector3d::Zero();
  ba = Eigen::Vector3d::Zero();
  scale = 1.0;

  /***************************
   * Build list of imu frames *
   ****************************/
  std::queue<sensor_msgs::Imu> imu_buffer_copy = imu_buffer;
  ros::Time start = path.poses[0].header.stamp;
  ros::Time end = path.poses[path.poses.size() - 1].header.stamp;
  std::vector<bs_common::ImuState> imu_frames;
  for (auto& pose : path.poses) {
    ros::Time stamp = pose.header.stamp;
    if (stamp < imu_buffer_copy.front().header.stamp) continue;

    // add imu data to frames preintegrator
    bs_common::PreIntegrator preintegrator;
    preintegrator.cov_w = params.cov_gyro_noise;
    preintegrator.cov_a = params.cov_accel_noise;
    preintegrator.cov_bg = params.cov_gyro_bias;
    preintegrator.cov_ba = params.cov_accel_bias;
    while (imu_buffer_copy.front().header.stamp <= stamp &&
           !imu_buffer_copy.empty()) {
      bs_common::IMUData imu_data(imu_buffer_copy.front());
      preintegrator.data.push_back(imu_data);
      imu_buffer_copy.pop();
    }
    Eigen::Matrix4d T_WORLD_BASELINK;
    bs_common::PoseMsgToTransformationMatrix(pose, T_WORLD_BASELINK);
    Eigen::Vector3d p_WORLD_BASELINK;
    Eigen::Quaterniond q_WORLD_BASELINK;
    beam::TransformMatrixToQuaternionAndTranslation(
        T_WORLD_BASELINK, q_WORLD_BASELINK, p_WORLD_BASELINK);

    // create frame and add to valid frame vector
    bs_common::ImuState new_frame(stamp, q_WORLD_BASELINK, p_WORLD_BASELINK,
                                  preintegrator);
    imu_frames.push_back(new_frame);
  }

  /***************************
   *     Estimate Gyro Bias   *
   ****************************/
  for (size_t i = 0; i < imu_frames.size(); i++) {
    imu_frames[i].GetPreintegrator()->Integrate(imu_frames[i].Stamp(), bg, ba,
                                                true, false);
  }
  Eigen::Matrix3d A1 = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b1 = Eigen::Vector3d::Zero();
  for (size_t j = 1; j < imu_frames.size(); ++j) {
    const size_t i = j - 1;
    const Eigen::Quaterniond& dq = imu_frames[j].GetPreintegrator()->delta.q;
    const Eigen::Matrix3d& dq_dbg =
        imu_frames[j].GetPreintegrator()->jacobian.dq_dbg;
    A1 += dq_dbg.transpose() * dq_dbg;

    Eigen::Quaterniond tmp =
        (imu_frames[i].OrientationQuat() * dq).conjugate() *
        imu_frames[j].OrientationQuat();
    Eigen::Matrix3d tmp_R = tmp.normalized().toRotationMatrix();

    b1 += dq_dbg.transpose() * beam::RToLieAlgebra(tmp_R);
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd1(A1, Eigen::ComputeFullU |
                                                 Eigen::ComputeFullV);
  bg = svd1.solve(b1);

  /*****************************
   * Estimate gravity and scale *
   ******************************/
  for (size_t i = 0; i < imu_frames.size(); i++) {
    imu_frames[i].GetPreintegrator()->Integrate(imu_frames[i].Stamp(), bg, ba,
                                                true, false);
  }
  Eigen::Matrix4d A2 = Eigen::Matrix4d::Zero();
  Eigen::Vector4d b2 = Eigen::Vector4d::Zero();
  
  for (size_t j = 1; j + 1 < imu_frames.size(); ++j) {
    const size_t i = j - 1;
    const size_t k = j + 1;

    const bs_common::Delta& delta_ij = imu_frames[j].GetPreintegrator()->delta;
    const bs_common::Delta& delta_jk = imu_frames[k].GetPreintegrator()->delta;
    const bs_common::Jacobian& jacobian_ij =
        imu_frames[j].GetPreintegrator()->jacobian;
    const bs_common::Jacobian& jacobian_jk =
        imu_frames[k].GetPreintegrator()->jacobian;

    Eigen::Matrix<double, 3, 4> C;
    C.block<3, 3>(0, 0) = -0.5 * delta_ij.t.toSec() * delta_jk.t.toSec() *
                          (delta_ij.t.toSec() + delta_jk.t.toSec()) *
                          Eigen::Matrix3d::Identity();
    C.block<3, 1>(0, 3) =
        delta_ij.t.toSec() *
            (imu_frames[k].PositionVec() - imu_frames[j].PositionVec()) -
        delta_jk.t.toSec() *
            (imu_frames[j].PositionVec() - imu_frames[i].PositionVec());
    Eigen::Vector3d d =
        delta_ij.t.toSec() * (imu_frames[j].OrientationQuat() * delta_jk.p) +
        delta_ij.t.toSec() * delta_jk.t.toSec() *
            (imu_frames[i].OrientationQuat() * delta_ij.v) -
        delta_jk.t.toSec() * (imu_frames[i].OrientationQuat() * delta_ij.p);
    A2 += C.transpose() * C;
    b2 += C.transpose() * d;
  }
  Eigen::JacobiSVD<Eigen::Matrix4d> svd2(A2, Eigen::ComputeFullU |
                                                 Eigen::ComputeFullV);
  Eigen::Vector4d x = svd2.solve(b2);
  gravity = x.segment<3>(0).normalized() * GRAVITY_NOMINAL;
  scale = x(3);
  
  /***************************
   *    Estimate Accel Bias   *
   ****************************/
  for (size_t i = 0; i < imu_frames.size(); i++) {
    imu_frames[i].GetPreintegrator()->Integrate(imu_frames[i].Stamp(), bg, ba,
                                                true, false);
  }
  Eigen::Matrix4d A3 = Eigen::Matrix4d::Zero();
  Eigen::Vector4d b3 = Eigen::Vector4d::Zero();
  for (size_t j = 1; j + 1 < imu_frames.size(); ++j) {
    const size_t i = j - 1;
    const size_t k = j + 1;

    const bs_common::Delta& delta_ij = imu_frames[j].GetPreintegrator()->delta;
    const bs_common::Delta& delta_jk = imu_frames[k].GetPreintegrator()->delta;
    const bs_common::Jacobian& jacobian_ij =
        imu_frames[j].GetPreintegrator()->jacobian;
    const bs_common::Jacobian& jacobian_jk =
        imu_frames[k].GetPreintegrator()->jacobian;

    Eigen::Matrix<double, 3, 4> C;
    C.block<3, 1>(0, 0) =
        delta_ij.t.toSec() *
            (imu_frames[k].PositionVec() - imu_frames[j].PositionVec()) -
        delta_jk.t.toSec() *
            (imu_frames[j].PositionVec() - imu_frames[i].PositionVec());
    C.block<3, 3>(0, 1) =
        -(imu_frames[j].OrientationQuat() * jacobian_jk.dp_dba *
              delta_ij.t.toSec() +
          imu_frames[i].OrientationQuat() * jacobian_ij.dv_dba *
              delta_ij.t.toSec() * delta_jk.t.toSec() -
          imu_frames[i].OrientationQuat() * jacobian_ij.dp_dba *
              delta_jk.t.toSec());
    Eigen::Vector3d d =
        0.5 * delta_ij.t.toSec() * delta_jk.t.toSec() *
            (delta_ij.t.toSec() + delta_jk.t.toSec()) * gravity +
        delta_ij.t.toSec() * (imu_frames[j].OrientationQuat() * delta_jk.p) +
        delta_ij.t.toSec() * delta_jk.t.toSec() *
            (imu_frames[i].OrientationQuat() * delta_ij.v) -
        delta_jk.t.toSec() * (imu_frames[i].OrientationQuat() * delta_ij.p);
    A3 += C.transpose() * C;
    b3 += C.transpose() * d;
  }
  Eigen::JacobiSVD<Eigen::Matrix4d> svd3(A3, Eigen::ComputeFullU |
                                                 Eigen::ComputeFullV);
  x = svd3.solve(b3);
  ba = x.segment<3>(1);
}

} // namespace bs_models
