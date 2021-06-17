#include <beam_models/frame_to_frame/preintegrator.h>

void PreIntegrator::Reset() {
  delta.t = 0;
  delta.q.setIdentity();
  delta.p.setZero();
  delta.v.setZero();
  delta.cov.setZero();
  delta.sqrt_inv_cov.setZero();

  jacobian.dq_dbg.setZero();
  jacobian.dp_dbg.setZero();
  jacobian.dp_dba.setZero();
  jacobian.dv_dbg.setZero();
  jacobian.dv_dba.setZero();
}

void PreIntegrator::Increment(double dt, const IMUData& data,
                              const Eigen::Vector3d& bg, const Vector3d& ba,
                              bool compute_jacobian, bool compute_covariance) {
  assert(("dt must > 0") && (dt >= 0));

  Eigen::Vector3d w = data.w - bg;
  Eigen::Vector3d a = data.a - ba;

  if (compute_covariance) {
    Eigen::Matrix<double, 9, 9> A;
    A.setIdentity();
    A.block<3, 3>(ES_Q, ES_Q) = beam::expmap(w * dt).conjugate().matrix();
    A.block<3, 3>(ES_V, ES_Q) = -dt * delta.q.matrix() * hat(a);
    A.block<3, 3>(ES_P, ES_Q) = -0.5 * dt * dt * delta.q.matrix() * hat(a);
    A.block<3, 3>(ES_P, ES_V) = dt * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 9, 6> B;
    B.setZero();
    B.block<3, 3>(ES_Q, ES_BG - ES_BG) = dt * beam::RightJacobian(w * dt);
    B.block<3, 3>(ES_V, ES_BA - ES_BG) = dt * delta.q.matrix();
    B.block<3, 3>(ES_P, ES_BA - ES_BG) = 0.5 * dt * dt * delta.q.matrix();

    Eigen::Matrix<double, 6, 6> white_noise_cov;
    double inv_dt = 1.0 / std::max(dt, 1.0e-7);
    white_noise_cov.setZero();
    white_noise_cov.block<3, 3>(ES_BG - ES_BG, ES_BG - ES_BG) = cov_w * inv_dt;
    white_noise_cov.block<3, 3>(ES_BA - ES_BG, ES_BA - ES_BG) = cov_a * inv_dt;

    delta.cov.block<9, 9>(ES_Q, ES_Q) =
        A * delta.cov.block<9, 9>(0, 0) * A.transpose() +
        B * white_noise_cov * B.transpose();
    delta.cov.block<3, 3>(ES_BG, ES_BG) += cov_bg * dt;
    delta.cov.block<3, 3>(ES_BA, ES_BA) += cov_ba * dt;
  }

  if (compute_jacobian) {
    jacobian.dp_dbg += dt * jacobian.dv_dbg - 0.5 * dt * dt * delta.q.matrix() *
                                                  hat(a) * jacobian.dq_dbg;
    jacobian.dp_dba += dt * jacobian.dv_dba - 0.5 * dt * dt * delta.q.matrix();
    jacobian.dv_dbg -= dt * delta.q.matrix() * hat(a) * jacobian.dq_dbg;
    jacobian.dv_dba -= dt * delta.q.matrix();
    jacobian.dq_dbg =
        beam::expmap(w * dt).conjugate().matrix() * jacobian.dq_dbg -
        dt * beam::RightJacobian(w * dt);
  }

  Eigen::Quaterniond q_intermediate = delta.q * beam::expmap(0.5 * w * dt);
  Eigen::Vector3d a_intermediate = q_intermediate * a;

  delta.t = delta.t + dt;
  delta.p = delta.p + dt * delta.v + 0.5 * dt * dt * a_intermediate;
  delta.v = delta.v + dt * a_intermediate;
  delta.q = (delta.q * beam::expmap(w * dt)).normalized();
}

bool PreIntegrator::Integrate(double t, const Eigen::Vector3d& bg,
                              const Vector3d& ba, bool compute_jacobian,
                              bool compute_covariance) {
  if (data.size() == 0) return false;
  this->Reset();
  for (size_t i = 0; i + 1 < data.size(); ++i) {
    const IMUData& d = data[i];
    this->Increment(data[i + 1].t - d.t, d, bg, ba, compute_jacobian,
                    compute_covariance);
  }
  assert(("Image time cannot be earlier than last imu.") &&
         (t >= data.back().t));
  this->Increment(t - data.back().t, data.back(), bg, ba, compute_jacobian,
                  compute_covariance);
  if (compute_covariance) { this->ComputeSqrtInverseCovariance(); }
  return true;
}

void PreIntegrator::ComputeSqrtInverseCovariance() {
  delta.sqrt_inv_cov =
      Eigen::LLT<Eigen::Matrix<double, 15, 15>>(delta.cov.inverse())
          .matrixL()
          .transpose();
}
