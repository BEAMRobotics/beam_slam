#include <bs_models/scan_registration/registration_validation.h>

#include <math.h>

#include <beam_utils/log.h>

#include <bs_common/utils.h>

namespace bs_models { namespace scan_registration {

bool RegistrationValidation::Validate(
    const Eigen::Matrix4d& T_measured,
    const Eigen::Matrix<double, 6, 6>& covariance) {
  Eigen::Matrix3d R = T_measured.block(0, 0, 3, 3);
  Eigen::AngleAxisd aa(R);
  RegistrationMetrics m;
  m.r = aa.angle();
  m.t = T_measured.block(0, 3, 3, 1).norm();
  m.entropy = bs_common::ShannonEntropyFromPoseCovariance(covariance);

  metrics_.push_back(m);

  if (metrics_.size() > list_size_) {
    metrics_.pop_front();
    return CheckStoredMetrics();
  } else {
    return CheckMetricInitial(m);
  }
}

bool RegistrationValidation::CheckStoredMetrics() const {
  double r_sum{0};
  double t_sum{0};
  double entropy_sum{0};

  for (const auto& m : metrics_) {
    r_sum += m.r;
    t_sum += m.t;
    entropy_sum += m.entropy;
  }

  double r_mean = r_sum / list_size_;
  double t_mean = t_sum / list_size_;
  double entropy_mean = entropy_sum / list_size_;

  double r_diffsqrd;
  double t_diffsqrd;
  double entropy_diffsqrd;
  for (const auto& m : metrics_) {
    r_diffsqrd += (m.r - r_mean) * (m.r - r_mean);
    t_diffsqrd += (m.t - t_mean) * (m.t - t_mean);
    entropy_diffsqrd += (m.entropy - entropy_mean) * (m.entropy - entropy_mean);
  }

  double r_stddev = std::sqrt(r_diffsqrd / list_size_);
  double t_stddev = std::sqrt(t_diffsqrd / list_size_);
  double entropy_stddev = std::sqrt(entropy_diffsqrd / list_size_);

  const RegistrationMetrics& m_recent = metrics_.back();
  if (m_recent.r > r_mean + 2 * r_stddev) {
    BEAM_WARN(
        "failed rotation check. Rotation = {} > mean + 2 x Std. Dev. = {}",
        m_recent.r, r_mean + 2 * r_stddev);
    return false;
  } else if (m_recent.r < r_mean - 2 * r_stddev) {
    BEAM_WARN(
        "failed rotation check. Rotation = {} < mean - 2 x Std. Dev. = {}",
        m_recent.r, r_mean - 2 * r_stddev);
    return false;
  } else if (m_recent.t > t_mean + 2 * t_stddev) {
    BEAM_WARN("failed translation check. Translation = {} > mean + 2 x Std. "
              "Dev. = {}",
              m_recent.t, t_mean + 2 * t_stddev);
    return false;
  } else if (m_recent.t < t_mean - 2 * t_stddev) {
    BEAM_WARN("failed translation check. Translation = {} < mean - 2 x Std. "
              "Dev. = {}",
              m_recent.t, t_mean - 2 * t_stddev);
    return false;
  } else if (m_recent.entropy > entropy_mean + 2 * entropy_stddev) {
    BEAM_WARN("failed translation check. Translation = {} > mean + 2 x Std. "
              "Dev. = {}",
              m_recent.entropy, entropy_mean + 2 * entropy_stddev);
    return false;
  } else if (m_recent.entropy < entropy_mean - 2 * entropy_stddev) {
    BEAM_WARN("failed translation check. Translation = {} < mean - 2 x Std. "
              "Dev. = {}",
              m_recent.entropy, entropy_mean - 2 * entropy_stddev);
    return false;
  }

  return true;
}

bool RegistrationValidation::CheckMetricInitial(
    const RegistrationMetrics& m) const {
  if (m.t > t_init_thresh_) {
    BEAM_WARN(
        "Registration translation result ({}) exceeds initial threshold ({})",
        m.t, t_init_thresh_);
    return false;
  } else if (m.r > r_init_thresh_) {
    BEAM_WARN(
        "Registration rotation result ({}) exceeds initial threshold ({})", m.t,
        t_init_thresh_);
    return false;
  } else if (m.entropy > entropy_init_thresh_) {
    BEAM_WARN("Registration entropy result ({}) exceeds initial threshold ({})",
              m.entropy, entropy_init_thresh_);
    return false;
  }

  return true;
}

}} // namespace bs_models::scan_registration