#pragma once

#include <ostream>
#include <string>
#include <vector>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <Eigen/Dense>
#include <fuse_core/constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/macros.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

#include <beam_variables/imu_bias_stamped.h>

namespace beam_constraints { namespace global {

/**
 * @brief A constraint that represents either prior information about a 3D imu
 * state, or a direct measurement of the 3D imu state.
 *
 * This constraint holds the measured 3D imu state and the measurement
 * uncertainty/covariance. Orientations are represented as quaternions.
 */
class AbsoluteImuState3DStampedConstraint : public fuse_core::Constraint {
public:
  FUSE_CONSTRAINT_DEFINITIONS_WITH_EIGEN(AbsoluteImuState3DStampedConstraint);

  AbsoluteImuState3DStampedConstraint() = default;

  AbsoluteImuState3DStampedConstraint(
      const std::string& source,
      const fuse_variables::Orientation3DStamped& orientation,
      const fuse_variables::Position3DStamped& position,
      const fuse_variables::VelocityLinear3DStamped& velocity, 
      const beam_variables::ImuBiasStamped& gyrobias,
      const beam_variables::ImuBiasStamped& accelbias,
      const Eigen::Matrix<double, 16, 1>& mean,
      const Eigen::Matrix<double, 15, 15>& covariance);

  virtual ~AbsoluteImuState3DStampedConstraint() = default;

  const Eigen::Matrix<double, 16, 1>& mean() const { return mean_; }

  const Eigen::Matrix<double, 15, 15>& sqrtInformation() const {
    return sqrt_information_;
  }

  Eigen::Matrix<double, 15, 15> covariance() const {
    return (sqrt_information_.transpose() * sqrt_information_).inverse();
  }

  void print(std::ostream& stream = std::cout) const override;

  ceres::CostFunction* costFunction() const override;

protected:
  Eigen::Matrix<double, 16, 1> mean_;
  Eigen::Matrix<double, 15, 15> sqrt_information_;

private:
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& archive, const unsigned int /* version */) {
    archive& boost::serialization::base_object<fuse_core::Constraint>(*this);
    archive& mean_;
    archive& sqrt_information_;
  }
};

}}  // namespace beam_constraints::global

BOOST_CLASS_EXPORT_KEY(
    beam_constraints::global::AbsoluteImuState3DStampedConstraint);
