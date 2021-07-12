#include <beam_models/initialization/imu_initializer.h>
#define GRAVITY_NOMINAL 9.86055

namespace beam_models { namespace camera_to_camera {

IMUInitializer::IMUInitializer(
    const std::vector<beam_models::camera_to_camera::Frame>& frames)
    : frames_(frames) {
  bg_.setZero();
  ba_.setZero();
  gravity_.setZero();
  scale_ = 1;
}

void IMUInitializer::Integrate() {
  for (size_t i = 0; i < frames_.size(); i++) {
    frames_[i].preint.Integrate(frames_[i].t, bg_, ba_, true, false);
  }
}

void IMUInitializer::SolveGyroBias() {
  Integrate();
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();

  for (size_t j = 1; j < frames_.size(); ++j) {
    const size_t i = j - 1;

    beam_models::camera_to_camera::Frame frame_i = frames_[i];
    beam_models::camera_to_camera::Frame frame_j = frames_[j];

    const Eigen::Quaterniond& dq = frames_[j].preint.delta.q;
    const Eigen::Matrix3d& dq_dbg = frames_[j].preint.jacobian.dq_dbg;
    A += dq_dbg.transpose() * dq_dbg;

    Eigen::Quaterniond tmp = (frames_[i].q * dq).conjugate() * frames_[j].q;
    Eigen::Matrix3d tmp_R = tmp.normalized().toRotationMatrix();

    b += dq_dbg.transpose() * beam::RToLieAlgebra(tmp_R);
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  bg_ = svd.solve(b);
}

void IMUInitializer::SolveAccelBias() {
  if (gravity_.isZero(1e-9)) {
    ROS_WARN(
        "Can't estimate acceleration bias without first estimating gravity.");
    return;
  }
  Integrate();
  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
  Eigen::Vector4d b = Eigen::Vector4d::Zero();

  for (size_t j = 1; j + 1 < frames_.size(); ++j) {
    const size_t i = j - 1;
    const size_t k = j + 1;

    const beam_common::Delta& delta_ij = frames_[j].preint.delta;
    const beam_common::Delta& delta_jk = frames_[k].preint.delta;
    const beam_common::Jacobian& jacobian_ij = frames_[j].preint.jacobian;
    const beam_common::Jacobian& jacobian_jk = frames_[k].preint.jacobian;

    Eigen::Matrix<double, 3, 4> C;
    C.block<3, 1>(0, 0) = delta_ij.t.toSec() * (frames_[k].p - frames_[j].p) -
                          delta_jk.t.toSec() * (frames_[j].p - frames_[i].p);
    C.block<3, 3>(0, 1) =
        -(frames_[j].q * jacobian_jk.dp_dba * delta_ij.t.toSec() +
          frames_[i].q * jacobian_ij.dv_dba * delta_ij.t.toSec() *
              delta_jk.t.toSec() -
          frames_[i].q * jacobian_ij.dp_dba * delta_jk.t.toSec());
    Eigen::Vector3d d =
        0.5 * delta_ij.t.toSec() * delta_jk.t.toSec() *
            (delta_ij.t.toSec() + delta_jk.t.toSec()) * gravity_ +
        delta_ij.t.toSec() * (frames_[j].q * delta_jk.p) +
        delta_ij.t.toSec() * delta_jk.t.toSec() * (frames_[i].q * delta_ij.v) -
        delta_jk.t.toSec() * (frames_[i].q * delta_ij.p);
    A += C.transpose() * C;
    b += C.transpose() * d;
  }

  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Vector4d x = svd.solve(b);
  ba_ = x.segment<3>(1);
}

void IMUInitializer::SolveGravityAndScale() {
  Integrate();
  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
  Eigen::Vector4d b = Eigen::Vector4d::Zero();

  for (size_t j = 1; j + 1 < frames_.size(); ++j) {
    const size_t i = j - 1;
    const size_t k = j + 1;

    const beam_common::Delta& delta_ij = frames_[j].preint.delta;
    const beam_common::Delta& delta_jk = frames_[k].preint.delta;
    const beam_common::Jacobian& jacobian_ij = frames_[j].preint.jacobian;
    const beam_common::Jacobian& jacobian_jk = frames_[k].preint.jacobian;

    Eigen::Matrix<double, 3, 4> C;
    C.block<3, 3>(0, 0) = -0.5 * delta_ij.t.toSec() * delta_jk.t.toSec() *
                          (delta_ij.t.toSec() + delta_jk.t.toSec()) *
                          Eigen::Matrix3d::Identity();
    C.block<3, 1>(0, 3) = delta_ij.t.toSec() * (frames_[k].p - frames_[j].p) -
                          delta_jk.t.toSec() * (frames_[j].p - frames_[i].p);
    Eigen::Vector3d d =
        delta_ij.t.toSec() * (frames_[j].q * delta_jk.p) +
        delta_ij.t.toSec() * delta_jk.t.toSec() * (frames_[i].q * delta_ij.v) -
        delta_jk.t.toSec() * (frames_[i].q * delta_ij.p);
    A += C.transpose() * C;
    b += C.transpose() * d;
  }

  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Vector4d x = svd.solve(b);
  gravity_ = x.segment<3>(0).normalized() * GRAVITY_NOMINAL;
  scale_ = x(3);
}

void IMUInitializer::RefineGravityAndScale() {
  static const double damp = 0.1;
  Integrate();
  int N = (int)frames_.size();
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  Eigen::VectorXd x;
  A.resize((N - 1) * 6, 2 + 1 + 3 * N);
  b.resize((N - 1) * 6);
  x.resize(2 + 1 + 3 * N);

  for (size_t iter = 0; iter < 5; ++iter) {
    A.setZero();
    b.setZero();
    Eigen::Matrix<double, 3, 2> Tg = s2_tangential_basis(gravity_);

    for (size_t j = 1; j < frames_.size(); ++j) {
      const size_t i = j - 1;

      const beam_common::Delta& delta = frames_[j].preint.delta;

      A.block<3, 2>(i * 6, 0) = -0.5 * delta.t.toSec() * delta.t.toSec() * Tg;
      A.block<3, 1>(i * 6, 2) = frames_[j].p - frames_[i].p;
      A.block<3, 3>(i * 6, 3 + i * 3) =
          -delta.t.toSec() * Eigen::Matrix3d::Identity();
      b.segment<3>(i * 6) = 0.5 * delta.t.toSec() * delta.t.toSec() * gravity_ +
                            frames_[i].q * delta.p;

      A.block<3, 2>(i * 6 + 3, 0) = -delta.t.toSec() * Tg;
      A.block<3, 3>(i * 6 + 3, 3 + i * 3) = -Eigen::Matrix3d::Identity();
      A.block<3, 3>(i * 6 + 3, 3 + j * 3) = Eigen::Matrix3d::Identity();
      b.segment<3>(i * 6 + 3) =
          delta.t.toSec() * gravity_ + frames_[i].q * delta.v;
    }

    x = A.fullPivHouseholderQr().solve(b);
    Eigen::Vector2d dg = x.segment<2>(0);
    gravity_ = (gravity_ + damp * Tg * dg).normalized() * GRAVITY_NOMINAL;
  }
  scale_ = x(2);
}

const Eigen::Vector3d& IMUInitializer::GetGyroBias() {
  return bg_;
}

const Eigen::Vector3d& IMUInitializer::GetAccelBias() {
  return ba_;
}

const Eigen::Vector3d& IMUInitializer::GetGravity() {
  return gravity_;
}

const double& IMUInitializer::GetScale() {
  return scale_;
}

}} // namespace beam_models::camera_to_camera