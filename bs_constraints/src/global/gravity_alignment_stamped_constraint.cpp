#include <bs_constraints/global/gravity_alignment_cost_functor.h>
#include <bs_constraints/global/gravity_alignment_stamped_constraint.h>

#include <Eigen/Dense>
#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>
#include <pluginlib/class_list_macros.h>

namespace bs_constraints { namespace global {

GravityAlignmentStampedConstraint::GravityAlignmentStampedConstraint(
    const std::string& source, const fuse_core::UUID& orientation_uuid,
    const Eigen::Vector3d& gravity_in_baselink,
    const Eigen::Matrix<double, 2, 2>& covariance)
    : fuse_core::Constraint(source, {orientation_uuid}),
      gravity_in_baselink_(gravity_in_baselink),
      sqrt_information_(covariance.inverse().llt().matrixU()) {}

void GravityAlignmentStampedConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  Orientation variable: " << variables().at(0) << "\n"
         << "  gravity_in_baselink_: [" << gravity_in_baselink_.transpose()
         << "]\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";

  if (loss()) {
    stream << "  loss: ";
    loss()->print(stream);
  }
}

ceres::CostFunction* GravityAlignmentStampedConstraint::costFunction() const {
  return new ceres::AutoDiffCostFunction<GravityAlignmentCostFunctor, 2, 4>(
      new GravityAlignmentCostFunctor(sqrt_information_, gravity_in_baselink_));
}

}} // namespace bs_constraints::global

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::global::GravityAlignmentStampedConstraint);
PLUGINLIB_EXPORT_CLASS(
    bs_constraints::global::GravityAlignmentStampedConstraint,
    fuse_core::Constraint);
