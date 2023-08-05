#include <bs_common/preintegrator.h>

#include <algorithm>

namespace bs_common {

void PreIntegrator::Reset() {
  delta.t = ros::Duration(0.0);
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

void PreIntegrator::Clear(const ros::Time& t) {
  auto leq_time = [&](const auto& d) { return d.t < t; };
  auto it = std::remove_if(data.begin(), data.end(), leq_time);
  auto r = std::distance(it, data.end());
  data.erase(it, data.end());
}

void PreIntegrator::Increment(const ros::Duration& dt, const IMUData& data,
                              const Eigen::Vector3d& bg,
                              const Eigen::Vector3d& ba, bool compute_jacobian,
                              bool compute_covariance) {
  assert(("dt must > 0") && (dt > ros::Duration(0)));
  double dtd = dt.toSec();
  Eigen::Vector3d w = data.w - bg;
  Eigen::Vector3d a = data.a - ba;

  Eigen::Quaterniond q_full(beam::LieAlgebraToR(w * dtd));
  Eigen::Quaterniond q_half(beam::LieAlgebraToR(0.5 * w * dtd));

  if (compute_covariance) {
    Eigen::Matrix<double, 9, 9> A;
    A.setIdentity();

    A.block<3, 3>(ES_Q, ES_Q) = q_full.conjugate().matrix();
    A.block<3, 3>(ES_V, ES_Q) =
        -dtd * delta.q.matrix() * beam::SkewTransform(a);
    A.block<3, 3>(ES_P, ES_Q) =
        -0.5 * dtd * dtd * delta.q.matrix() * beam::SkewTransform(a);
    A.block<3, 3>(ES_P, ES_V) = dtd * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 9, 6> B;
    B.setZero();
    B.block<3, 3>(ES_Q, ES_BG - ES_BG) =
        dtd * beam::RightJacobianOfSO3(w * dtd);
    B.block<3, 3>(ES_V, ES_BA - ES_BG) = dtd * delta.q.matrix();
    B.block<3, 3>(ES_P, ES_BA - ES_BG) = 0.5 * dtd * dtd * delta.q.matrix();

    Eigen::Matrix<double, 6, 6> white_noise_cov;
    double inv_dtd = 1.0 / std::max(dtd, 1.0e-7);
    white_noise_cov.setZero();
    white_noise_cov.block<3, 3>(ES_BG - ES_BG, ES_BG - ES_BG) = cov_w * inv_dtd;
    white_noise_cov.block<3, 3>(ES_BA - ES_BG, ES_BA - ES_BG) = cov_a * inv_dtd;

    delta.cov.block<9, 9>(ES_Q, ES_Q) =
        A * delta.cov.block<9, 9>(0, 0) * A.transpose() +
        B * white_noise_cov * B.transpose();
    delta.cov.block<3, 3>(ES_BG, ES_BG) += cov_bg * dtd;
    delta.cov.block<3, 3>(ES_BA, ES_BA) += cov_ba * dtd;
  }

  if (compute_jacobian) {
    jacobian.dp_dbg +=
        dtd * jacobian.dv_dbg - 0.5 * dtd * dtd * delta.q.matrix() *
                                    beam::SkewTransform(a) * jacobian.dq_dbg;
    jacobian.dp_dba +=
        dtd * jacobian.dv_dba - 0.5 * dtd * dtd * delta.q.matrix();
    jacobian.dv_dbg -=
        dtd * delta.q.matrix() * beam::SkewTransform(a) * jacobian.dq_dbg;
    jacobian.dv_dba -= dtd * delta.q.matrix();
    jacobian.dq_dbg = q_full.conjugate().matrix() * jacobian.dq_dbg -
                      dtd * beam::RightJacobianOfSO3(w * dtd);
  }

  Eigen::Quaterniond q_mid = delta.q * q_half;
  Eigen::Vector3d a_mid = q_mid * a;

  delta.t = delta.t + dt;
  delta.p = delta.p + dtd * delta.v + 0.5 * dtd * dtd * a_mid;
  delta.v = delta.v + dtd * a_mid;
  delta.q = (delta.q * q_full).normalized();
}

bool PreIntegrator::Integrate(const ros::Time& t, const Eigen::Vector3d& bg,
                              const Eigen::Vector3d& ba, bool compute_jacobian,
                              bool compute_covariance) {
  if (data.empty()) return false;
  Reset();
  
  // increment over window such that it is less or equal to the requested time
  for (int i = 0; i < data.size(); i++) {
    const int j = i + 1;
    if (j >= data.size()) { break; }
    const auto cur = data[i];
    const auto next = data[j];
    if (next.t > t) { break; }
    const auto dt = next.t - cur.t;
    Increment(dt, cur, bg, ba, compute_jacobian, compute_covariance);
  }
  
  // final increment to requested time
  const auto dt = t - data.rbegin()->t;
  if (dt > ros::Duration(0)) {
    Increment(dt, *data.rbegin(), bg, ba, compute_jacobian, compute_covariance);
  }
  
  if (compute_covariance) { ComputeSqrtInvCov(); }
  
  return true;
}

void PreIntegrator::ComputeSqrtInvCov() {
  // Ensure covariance non-zero (within pre-defined tolarance) to avoid
  // ill-conditioned matrix during optimization

  // upper left
  const auto norm1 = delta.cov.block<9, 9>(ES_Q, ES_Q).norm();
  if (norm1 < cov_tol) {
    delta.cov.block<9, 9>(ES_Q, ES_Q).setIdentity();
    delta.cov.block<9, 9>(ES_Q, ES_Q) *= cov_tol;
  }

  // bottom right
  const auto norm2 = delta.cov.block<6, 6>(ES_BG, ES_BG).norm();
  if (norm2 < bias_cov_tol) {
    delta.cov.block<6, 6>(ES_BG, ES_BG).setIdentity();
    delta.cov.block<6, 6>(ES_BG, ES_BG) *= bias_cov_tol;
  }

  delta.sqrt_inv_cov =
      Eigen::LLT<Eigen::Matrix<double, ES_SIZE, ES_SIZE>>(delta.cov.inverse())
          .matrixL()
          .transpose();
}

} // namespace bs_common
