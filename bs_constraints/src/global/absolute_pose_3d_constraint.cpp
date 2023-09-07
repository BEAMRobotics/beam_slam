#include <bs_constraints/global/absolute_pose_3d_constraint.h>

#include <fuse_constraints/normal_prior_pose_3d_cost_functor.h>
#include <pluginlib/class_list_macros.h>

#include <Eigen/Dense>
#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>

namespace bs_constraints {

AbsolutePose3DConstraint::AbsolutePose3DConstraint(
    const std::string& source, const bs_variables::Position3D& position,
    const bs_variables::Orientation3D& orientation,
    const fuse_core::Vector7d& mean, const fuse_core::Matrix6d& covariance)
    : fuse_core::Constraint(
          source,
          {position.uuid(), orientation.uuid()}), // NOLINT(whitespace/braces)
      mean_(mean),
      sqrt_information_(covariance.inverse().llt().matrixU()) {}

void AbsolutePose3DConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  position variable: " << variables().at(0) << "\n"
         << "  orientation variable: " << variables().at(1) << "\n"
         << "  mean: " << mean().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";

  if (loss()) {
    stream << "  loss: ";
    loss()->print(stream);
  }
}

ceres::CostFunction* AbsolutePose3DConstraint::costFunction() const {
  return new ceres::AutoDiffCostFunction<
      fuse_constraints::NormalPriorPose3DCostFunctor, 6, 3, 4>(
      new fuse_constraints::NormalPriorPose3DCostFunctor(sqrt_information_,
                                                         mean_));
}

} // namespace bs_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(bs_constraints::AbsolutePose3DConstraint);
PLUGINLIB_EXPORT_CLASS(bs_constraints::AbsolutePose3DConstraint,
                       fuse_core::Constraint);
