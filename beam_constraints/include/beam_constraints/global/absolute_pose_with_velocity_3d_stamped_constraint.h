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

namespace beam_constraints { namespace global {

/**
 * @brief A constraint that represents either prior information about a 3D pose
 * with velocity, or a direct measurement of the 3D pose with velocity.
 *
 * This constraint holds the measured 3D pose with velocity and the measurement
 * uncertainty/covariance. Orientations are represented as quaternions.
 */
class AbsolutePoseWithVelocity3DStampedConstraint
    : public fuse_core::Constraint {
public:
  FUSE_CONSTRAINT_DEFINITIONS_WITH_EIGEN(
      AbsolutePoseWithVelocity3DStampedConstraint);

  AbsolutePoseWithVelocity3DStampedConstraint() = default;

  AbsolutePoseWithVelocity3DStampedConstraint(
      const std::string& source,
      const fuse_variables::Position3DStamped& position,
      const fuse_variables::VelocityLinear3DStamped& velocity,
      const fuse_variables::Orientation3DStamped& orientation,
      const Eigen::Matrix<double, 10, 1>& mean,
      const fuse_core::Matrix9d& covariance);

  virtual ~AbsolutePoseWithVelocity3DStampedConstraint() = default;

  const Eigen::Matrix<double, 10, 1>& mean() const { return mean_; }

  const fuse_core::Matrix9d& sqrtInformation() const {
    return sqrt_information_;
  }

  fuse_core::Matrix9d covariance() const {
    return (sqrt_information_.transpose() * sqrt_information_).inverse();
  }

  void print(std::ostream& stream = std::cout) const override;

  ceres::CostFunction* costFunction() const override;

protected:
  Eigen::Matrix<double, 10, 1> mean_;
  fuse_core::Matrix9d sqrt_information_;

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
    beam_constraints::global::AbsolutePoseWithVelocity3DStampedConstraint);