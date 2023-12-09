#include <bs_models/vision/vo_localization_validation.h>

#include <math.h>

#include <beam_utils/log.h>

#include <bs_common/utils.h>

namespace bs_models { namespace vision {

bool VOLocalizationValidation::Validate(
    const Eigen::Matrix4d& T_measured,
    const Eigen::Matrix<double, 6, 6>& covariance,
    const double avg_reprojection) {
  Eigen::Matrix3d R = T_measured.block(0, 0, 3, 3);
  Eigen::AngleAxisd aa(R);
  VOLocalizationMetrics m;
  m.r = aa.angle();
  m.t = T_measured.block(0, 3, 3, 1).norm();
  m.entropy = bs_common::ShannonEntropyFromPoseCovariance(covariance);
  m.avg_reprojection = avg_reprojection;

  metrics_.push_back(m);
  bool passed;
  if (metrics_.size() > list_size_) {
    metrics_.pop_front();
    passed = CheckStoredMetrics();
  } else {
    passed = CheckMetricInitial(m);
  }
  return passed;
}

bool VOLocalizationValidation::CheckStoredMetrics() const {
  double r_sum{0};
  double t_sum{0};
  double entropy_sum{0};
  double reproj_sum{0};

  for (const auto& m : metrics_) {
    r_sum += m.r;
    t_sum += m.t;
    entropy_sum += m.entropy;
    reproj_sum += m.avg_reprojection;
  }

  double r_mean = r_sum / list_size_;
  double t_mean = t_sum / list_size_;
  double entropy_mean = entropy_sum / list_size_;
  double reproj_mean = reproj_sum / list_size_;

  double r_diffsqrd;
  double t_diffsqrd;
  double entropy_diffsqrd;
  double reproj_diffsqrd;
  for (const auto& m : metrics_) {
    r_diffsqrd += (m.r - r_mean) * (m.r - r_mean);
    t_diffsqrd += (m.t - t_mean) * (m.t - t_mean);
    entropy_diffsqrd += (m.entropy - entropy_mean) * (m.entropy - entropy_mean);
    reproj_diffsqrd +=
        (m.avg_reprojection - reproj_mean) * (m.avg_reprojection - reproj_mean);
  }

  double r_stddev = std::sqrt(r_diffsqrd / list_size_);
  double t_stddev = std::sqrt(t_diffsqrd / list_size_);
  double entropy_stddev = std::sqrt(entropy_diffsqrd / list_size_);
  double reproj_stddev = std::sqrt(reproj_diffsqrd / list_size_);

  const VOLocalizationMetrics& m_recent = metrics_.back();
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
  } else if (m_recent.entropy > entropy_mean + 5 * entropy_stddev) {
    BEAM_WARN("failed entropy check. entropy = {} > mean + 5 x Std. "
              "Dev. = {}",
              m_recent.entropy, entropy_mean + 5 * entropy_stddev);
    return false;
  } else if (m_recent.entropy < entropy_mean - 5 * entropy_stddev) {
    BEAM_WARN("failed entropy check. entropy = {} < mean - 5 x Std. "
              "Dev. = {}",
              m_recent.entropy, entropy_mean - 5 * entropy_stddev);
    return false;
  } else if (m_recent.avg_reprojection < reproj_mean - 5 * reproj_stddev) {
    BEAM_WARN("failed reprojection check. reprojection = {} < mean - 5 x Std. "
              "Dev. = {}",
              m_recent.avg_reprojection, reproj_mean - 5 * reproj_stddev);
    return false;
  } else if (m_recent.avg_reprojection > reproj_mean + 5 * reproj_stddev) {
    BEAM_WARN("failed reprojection check. reprojection = {} < mean - 5 x Std. "
              "Dev. = {}",
              m_recent.avg_reprojection, reproj_mean + 5 * reproj_stddev);
    return false;
  }

  return true;
}

bool VOLocalizationValidation::CheckMetricInitial(
    const VOLocalizationMetrics& m) const {
  if (m.t > t_init_thresh_) {
    BEAM_WARN("VO Localization translation result ({}) exceeds initial "
              "threshold ({})",
              m.t, t_init_thresh_);
    return false;
  } else if (m.r > r_init_thresh_) {
    BEAM_WARN(
        "VO Localization rotation result ({}) exceeds initial threshold ({})",
        m.t, t_init_thresh_);
    return false;
  } else if (m.entropy > entropy_init_thresh_) {
    BEAM_WARN(
        "VO Localization entropy result ({}) exceeds initial threshold ({})",
        m.entropy, entropy_init_thresh_);
    return false;
  }

  return true;
}

void VOLocalizationValidation::Clear() {
  metrics_.clear();
}

}} // namespace bs_models::vision