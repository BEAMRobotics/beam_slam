#pragma once

#include <list>

#include <Eigen/Dense>

namespace bs_models { namespace scan_registration {

/**
 * @brief This class is intended to detect outlier registrations. We assume that
 * an initial guess is provided to the scan registration which should be close
 * to the true pose, and this class receives the registration result after
 * aligning to the true pose (T_measured). Therefore, we can assume that
 * T_measured should be relatively consistent between recent results. There
 * should not have any major spikes for inlier results. This assumes that
 * initial estimates come from some odometry source that may suffer from drift
 * over time but no outlier poses. Similar can be said for the covariance, the
 * information gain from each measurement (calculated using entropy) should be
 * relatively consistent between recent results. Any big decreases in
 * information gain will get flagged as an outlier. For each metric, we classify
 * them as outliers if their metric is more than 2x the standard
 * deviation away from the mean of recent metrics
 *
 * Shannon Entropy: This can be interpreted geometrically as the volume of the
 * uncertainty. The smaller the Shannon Entropy, the more certain we are.
 *
 * H(x) = 0.5 ln[(2 PI e)^N det(cov)]
 *
 * Where N is the numer of parameters being estimated
 *
 */
class RegistrationValidation {
public:
  struct RegistrationMetrics {
    double r;
    double t;
    double entropy;
  };

  RegistrationValidation() = default;

  ~RegistrationValidation() = default;

  bool Validate(const Eigen::Matrix4d& T_measured,
                const Eigen::Matrix<double, 6, 6>& covariance);

private:
  bool CheckStoredMetrics() const;

  bool CheckMetricInitial(const RegistrationMetrics& m) const;

  std::list<RegistrationMetrics> metrics_;
  int list_size_{15};

  // these are used when metrics_.size() < list_size_ since we don't have
  // enough data to get reliable statistics
  double t_init_thresh_{0.5};
  double r_init_thresh_{3.14 / 6};  // ~30deg
  double entropy_init_thresh_{-10}; // set empirically
};

}} // namespace bs_models::scan_registration
