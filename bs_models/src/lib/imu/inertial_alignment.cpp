#include <bs_models/imu/inertial_alignment.h>

namespace bs_models { namespace imu {
void EstimateParameters(const std::map<uint64_t, Eigen::Matrix4d>& path,
                        const std::deque<sensor_msgs::Imu>& imu_buffer,
                        const bs_models::ImuPreintegration::Params& params,
                        Eigen::Vector3d& gravity, Eigen::Vector3d& bg,
                        Eigen::Vector3d& ba,
                        std::map<uint64_t, Eigen::Vector3d>& velocities,
                        double& scale) {
  // set parameter estimates to 0
  gravity = Eigen::Vector3d::Zero();
  bg = Eigen::Vector3d::Zero();
  ba = Eigen::Vector3d::Zero();
  scale = 1.0;
  velocities.clear();
  std::vector<std::pair<uint64_t, Eigen::Vector3d>> velocities_vec;

  // make copy of imu buffer
  std::deque<sensor_msgs::Imu> imu_buffer_copy = imu_buffer;

  // check for invalid input
  auto second_imu_msg = std::next(imu_buffer_copy.begin());
  if (path.begin()->first < second_imu_msg->header.stamp.toNSec()) {
    const std::string msg =
        std::string(__func__) +
        ": Pose in path has less than 2 messages prior to it.";
    ROS_ERROR_STREAM(msg);
    throw std::runtime_error{msg};
  }

  // build set of imu frames
  std::vector<bs_common::ImuState> imu_frames;
  for (auto& [time_nsec, T_WORLD_BASELINK] : path) {
    const auto stamp = beam::NSecToRos(time_nsec);

    // add imu data to frames preintegrator
    bs_common::PreIntegrator preintegrator;
    preintegrator.cov_w = params.cov_gyro_noise;
    preintegrator.cov_a = params.cov_accel_noise;
    preintegrator.cov_bg = params.cov_gyro_bias;
    preintegrator.cov_ba = params.cov_accel_bias;
    while (imu_buffer_copy.front().header.stamp < stamp &&
           !imu_buffer_copy.empty()) {
      preintegrator.data.push_back(imu_buffer_copy.front());
      imu_buffer_copy.pop_front();
    }
    if (preintegrator.data.empty()) {
      const std::string msg = std::string(__func__) +
                              ": Empty preintegrator for pose in init path.";
      ROS_ERROR_STREAM(msg);
      throw std::runtime_error{msg};
    }
    Eigen::Vector3d p_WORLD_BASELINK;
    Eigen::Quaterniond q_WORLD_BASELINK;
    beam::TransformMatrixToQuaternionAndTranslation(
        T_WORLD_BASELINK, q_WORLD_BASELINK, p_WORLD_BASELINK);

    // create frame and add to valid frame vector
    bs_common::ImuState new_frame(stamp, q_WORLD_BASELINK, p_WORLD_BASELINK,
                                  preintegrator);
    imu_frames.push_back(new_frame);
  }

  const int N = imu_frames.size();

  // initialize velocities as 0 and reserve memory
  velocities_vec.resize(N, {0, Eigen::Vector3d::Zero()});
  for (int i = 0; i < N; ++i) {
    velocities_vec[i] = {imu_frames[i].Stamp().toNSec(),
                         Eigen::Vector3d::Zero()};
  }

  // lambda to integrate each frame to its time
  auto integrate = [&](auto& imu_frame) {
    imu_frame.GetPreintegrator()->Integrate(imu_frame.Stamp(), bg, ba, true,
                                            false);
  };

  std::for_each(imu_frames.begin(), imu_frames.end(), integrate);
  const auto var = ImuObservability(imu_frames);
  if (var < 0.25) {
    ROS_INFO_STREAM(__func__ << ": IMU excitation not enough: " << var
                             << " < 0.25. Cannot initialize.");
    return;
  }

  std::for_each(imu_frames.begin(), imu_frames.end(), integrate);
  EstimateGyroBias(imu_frames, bg);

  std::for_each(imu_frames.begin(), imu_frames.end(), integrate);
  EstimateGravityScaleVelocities(imu_frames, gravity, scale, velocities_vec);

  // std::for_each(imu_frames.begin(), imu_frames.end(), integrate);
  // RefineGravityScaleVelocities(imu_frames, gravity, scale, velocities_vec);

  // convert velocities to map
  std::for_each(
      velocities_vec.begin(), velocities_vec.end(),
      [&](const auto& pair) { velocities[pair.first] = pair.second; });

  // clang-format off
  ROS_INFO_STREAM(
       __func__ << ": IMU Parameter Estimation Results:" << 
       "\n \tScale: " << scale <<  
       "\n \tGravity: [" << gravity.x() << ", " << gravity.y() << ", " << gravity.z() << "]" 
       "\n \tGyro Bias: [" << bg.x() << ", " << bg.y() << ", " << bg.z() << "]" 
       "\n\tAccel Bias: [" << ba.x() << ", " << ba.y() << ", " << ba.z() << "]" );
  // clang-format on
}

double ImuObservability(const std::vector<bs_common::ImuState>& imu_frames) {
  Eigen::Vector3d sum_g;
  for (size_t i = 0; i < imu_frames.size(); ++i) {
    const double dt = imu_frames[i].GetConstPreintegrator().delta.t.toSec();
    const Eigen::Vector3d tmp_g =
        imu_frames[i].GetConstPreintegrator().delta.v / dt;
    sum_g += tmp_g;
  }

  Eigen::Vector3d aver_g;
  aver_g = sum_g * 1.0 / ((int)imu_frames.size() - 1);
  double var = 0;
  for (size_t i = 0; i < imu_frames.size(); ++i) {
    const double dt = imu_frames[i].GetConstPreintegrator().delta.t.toSec();
    const Eigen::Vector3d tmp_g =
        imu_frames[i].GetConstPreintegrator().delta.v / dt;
    var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
  }

  var = sqrt(var / ((int)imu_frames.size() - 1));
  if (var < 0.25) { ROS_INFO("IMU excitation not enough!"); }
  return var;
}

void EstimateGyroBias(const std::vector<bs_common::ImuState>& imu_frames,
                      Eigen::Vector3d& bg) {
  const int N = imu_frames.size();
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();
  for (size_t j = 1; j < N; ++j) {
    const size_t i = j - 1;
    const Eigen::Quaterniond& dq =
        imu_frames[j].GetConstPreintegrator().delta.q;
    const Eigen::Matrix3d& dq_dbg =
        imu_frames[j].GetConstPreintegrator().jacobian.dq_dbg;
    A += dq_dbg.transpose() * dq_dbg;

    Eigen::Quaterniond tmp =
        (imu_frames[i].OrientationQuat() * dq).conjugate() *
        imu_frames[j].OrientationQuat();
    Eigen::Matrix3d tmp_R = tmp.normalized().toRotationMatrix();

    b += dq_dbg.transpose() * beam::RToLieAlgebra(tmp_R);
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  bg = svd.solve(b);
}

void EstimateGravityScaleVelocities(
    const std::vector<bs_common::ImuState>& imu_frames,
    Eigen::Vector3d& gravity, double& scale,
    std::vector<std::pair<uint64_t, Eigen::Vector3d>>& velocities) {
  const int N = imu_frames.size();

  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  A.resize((N - 1) * 6, 3 + 1 + 3 * N);
  b.resize((N - 1) * 6);
  A.setZero();
  b.setZero();

  for (size_t j = 1; j < N; ++j) {
    const size_t i = j - 1;

    const bs_common::Delta& delta_ij =
        imu_frames[j].GetConstPreintegrator().delta;

    A.block<3, 3>(i * 6, 0) = -0.5 * delta_ij.t.toSec() * delta_ij.t.toSec() *
                              Eigen::Matrix3d::Identity();
    A.block<3, 1>(i * 6, 3) =
        imu_frames[j].PositionVec() - imu_frames[i].PositionVec();
    A.block<3, 3>(i * 6, 4 + i * 3) =
        -delta_ij.t.toSec() * Eigen::Matrix3d::Identity();
    b.segment<3>(i * 6) = imu_frames[i].OrientationQuat() * delta_ij.p;
    A.block<3, 3>(i * 6 + 3, 0) =
        -delta_ij.t.toSec() * Eigen::Matrix3d::Identity();
    A.block<3, 3>(i * 6 + 3, 4 + i * 3) = -Eigen::Matrix3d::Identity();
    A.block<3, 3>(i * 6 + 3, 4 + j * 3) = Eigen::Matrix3d::Identity();
    b.segment<3>(i * 6 + 3) = imu_frames[i].OrientationQuat() * delta_ij.v;
  }

  Eigen::VectorXd x = A.fullPivHouseholderQr().solve(b);
  gravity = x.segment<3>(0).normalized() * GRAVITY_NOMINAL;
  scale = x(3);
  for (size_t i = 0; i < N; ++i) {
    velocities[i].second = x.segment<3>(4 + i * 3);
  }
}

void RefineGravityScaleVelocities(
    const std::vector<bs_common::ImuState>& imu_frames,
    Eigen::Vector3d& gravity, double& scale,
    std::vector<std::pair<uint64_t, Eigen::Vector3d>>& velocities) {
  const int N = imu_frames.size();
  static const double damp = 0.1;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  Eigen::VectorXd x;
  A.resize((N - 1) * 6, 2 + 1 + 3 * N);
  b.resize((N - 1) * 6);
  x.resize(2 + 1 + 3 * N);
  A.setZero();
  b.setZero();
  Eigen::Matrix<double, 3, 2> Tg = beam::S2TangentialBasis(gravity);

  for (size_t j = 1; j < N; ++j) {
    const size_t i = j - 1;

    const bs_common::Delta& delta = imu_frames[j].GetConstPreintegrator().delta;

    A.block<3, 2>(i * 6, 0) = -0.5 * delta.t.toSec() * delta.t.toSec() * Tg;
    A.block<3, 1>(i * 6, 2) =
        imu_frames[j].PositionVec() - imu_frames[i].PositionVec();
    A.block<3, 3>(i * 6, 3 + i * 3) =
        -delta.t.toSec() * Eigen::Matrix3d::Identity();
    b.segment<3>(i * 6) = 0.5 * delta.t.toSec() * delta.t.toSec() * gravity +
                          imu_frames[i].OrientationQuat() * delta.p;

    A.block<3, 2>(i * 6 + 3, 0) = -delta.t.toSec() * Tg;
    A.block<3, 3>(i * 6 + 3, 3 + i * 3) = -Eigen::Matrix3d::Identity();
    A.block<3, 3>(i * 6 + 3, 3 + j * 3) = Eigen::Matrix3d::Identity();
    b.segment<3>(i * 6 + 3) =
        delta.t.toSec() * gravity + imu_frames[i].OrientationQuat() * delta.v;

    x = A.fullPivHouseholderQr().solve(b);
    Eigen::Vector2d dg = x.segment<2>(0);
    gravity = (gravity + damp * Tg * dg).normalized() * GRAVITY_NOMINAL;
  }
  scale = x(2);
  for (size_t i = 0; i < N; ++i) {
    velocities[i].second = x.segment<3>(3 + i * 3);
  }
}

}} // namespace bs_models::imu