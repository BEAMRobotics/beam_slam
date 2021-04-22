#include <beam_constraints/global/absolute_imu_state_3d_stamped_constraint.h>
#include <beam_constraints/global/normal_prior_imu_state_3d_cost_functor.h>

#include <string>

#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>
#include <Eigen/Dense>
#include <pluginlib/class_list_macros.h>

namespace beam_constraints { namespace global {

AbsoluteImuState3DStampedConstraint::AbsoluteImuState3DStampedConstraint(
    const std::string& source,
    const fuse_variables::Position3DStamped& position,
    const fuse_variables::VelocityLinear3DStamped& velocity,
    const fuse_variables::Orientation3DStamped& orientation,
    const Eigen::Matrix<double, 10, 1>& mean,
    const fuse_core::Matrix9d& covariance)
    : fuse_core::Constraint(source,
                            {position.uuid(), velocity.uuid(),
                             orientation.uuid()}),  // NOLINT(whitespace/braces)
      mean_(mean),
      sqrt_information_(covariance.inverse().llt().matrixU()) {}

void AbsoluteImuState3DStampedConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  position variable: " << variables().at(0) << "\n"
         << "  velocity variable: " << variables().at(1) << "\n"
         << "  orientation variable: " << variables().at(2) << "\n"
         << "  mean: " << mean().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";

  if (loss()) {
    stream << "  loss: ";
    loss()->print(stream);
  }
}

ceres::CostFunction* AbsoluteImuState3DStampedConstraint::costFunction() const {
  return new ceres::AutoDiffCostFunction<NormalPriorImuState3DCostFunctor, 9, 3,
                                         3, 4>(
      new NormalPriorImuState3DCostFunctor(sqrt_information_, mean_));
}

}}  // namespace beam_constraints::global

BOOST_CLASS_EXPORT_IMPLEMENT(
    beam_constraints::global::AbsoluteImuState3DStampedConstraint);
PLUGINLIB_EXPORT_CLASS(
    beam_constraints::global::AbsoluteImuState3DStampedConstraint,
    fuse_core::Constraint);
