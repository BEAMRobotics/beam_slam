#include <bs_common/preintegrator.h>

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

void PreIntegrator::Increment(ros::Duration dt, const IMUData& data,
                              const Eigen::Vector3d& bg,
                              const Eigen::Vector3d& ba, bool compute_jacobian,
                              bool compute_covariance) {
  assert(("dt must > 0") && (dt >= ros::Duration(0)));
  double dtd = dt.toSec();
  Eigen::Vector3d w = data.w - bg;
  Eigen::Vector3d a = data.a - ba;

  Eigen::Quaterniond del_q_mid(beam::LieAlgebraToR(0.5 * w * dtd));
  del_q_mid.normalize();

  Eigen::Quaterniond q_mid = delta.q * del_q_mid;
  Eigen::Vector3d a_mid = q_mid * a;

  if (compute_covariance) {
    Eigen::Matrix<double, 9, 9> A;
    A.setIdentity();
    A.block<3, 3>(ES_Q, ES_Q) = del_q_mid.conjugate().matrix();
    A.block<3, 3>(ES_V, ES_Q) =
        -dtd * delta.q.matrix() * beam::SkewTransform(a_mid);
    A.block<3, 3>(ES_P, ES_Q) =
        -0.5 * dtd * dtd * delta.q.matrix() * beam::SkewTransform(a_mid);
    A.block<3, 3>(ES_P, ES_V) = dtd * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 9, 6> B;
    B.setZero();
    B.block<3, 3>(ES_Q, ES_BG - ES_BG) =
        dtd * beam::RightJacobianOfSO3(0.5 * w * dtd);
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
    jacobian.dp_dbg += dtd * jacobian.dv_dbg -
                       0.5 * dtd * dtd * delta.q.matrix() *
                           beam::SkewTransform(a_mid) * jacobian.dq_dbg;
    jacobian.dp_dba +=
        dtd * jacobian.dv_dba - 0.5 * dtd * dtd * delta.q.matrix();
    jacobian.dv_dbg -=
        dtd * delta.q.matrix() * beam::SkewTransform(a_mid) * jacobian.dq_dbg;
    jacobian.dv_dba -= dtd * delta.q.matrix();
    jacobian.dq_dbg = del_q_mid.conjugate().matrix() * jacobian.dq_dbg -
                      dtd * beam::RightJacobianOfSO3(0.5 * w * dtd);
  }

  delta.t = delta.t + dt;
  delta.p = delta.p + dtd * delta.v + 0.5 * dtd * dtd * a_mid;
  delta.v = delta.v + dtd * a_mid;
  delta.q = q_mid;
}

bool PreIntegrator::Integrate(ros::Time t, const Eigen::Vector3d& bg,
                              const Eigen::Vector3d& ba, bool compute_jacobian,
                              bool compute_covariance) {
  if (data.size() == 0) return false;
  Reset();
  for (size_t i = 0; i + 1 < data.size(); ++i) {
    const IMUData& d = data[i];
    Increment(data[i + 1].t - d.t, d, bg, ba, compute_jacobian,
              compute_covariance);
  }
  assert(("Frame time cannot be earlier than last imu.") &&
         (t >= data.back().t));
  Increment(t - data.back().t, data.back(), bg, ba, compute_jacobian,
            compute_covariance);
  if (compute_covariance) {
    ComputeSqrtInvCov();
  }
  return true;
}

void PreIntegrator::ComputeSqrtInvCov() {
  // Ensure covariance non-zero (within pre-defined tolarance) to avoid
  // ill-conditioned matrix during optimization
  if (delta.cov.block<9, 9>(ES_Q, ES_Q).isZero(cov_tol)) {
    delta.cov.block<9, 9>(ES_Q, ES_Q).setIdentity();
    delta.cov.block<9, 9>(ES_Q, ES_Q) *= cov_tol;
  }

  if (delta.cov.block<6, 6>(ES_BG, ES_BG).isZero(cov_tol)) {
    delta.cov.block<6, 6>(ES_BG, ES_BG).setIdentity();
    delta.cov.block<6, 6>(ES_BG, ES_BG) *= cov_tol;
  }

  delta.sqrt_inv_cov =
      Eigen::LLT<Eigen::Matrix<double, ES_SIZE, ES_SIZE>>(delta.cov.inverse())
          .matrixL()
          .transpose();
}

}  // namespace bs_common
