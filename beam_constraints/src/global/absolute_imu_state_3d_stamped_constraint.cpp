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
    const fuse_variables::Orientation3DStamped& orientation,
    const fuse_variables::Position3DStamped& position,
    const fuse_variables::VelocityLinear3DStamped& velocity,
    const beam_variables::ImuBiasGyro3DStamped& gyrobias,
    const beam_variables::ImuBiasAccel3DStamped& accelbias,
    const Eigen::Matrix<double, 16, 1>& mean,
    const Eigen::Matrix<double, 15, 15>& covariance)
    : fuse_core::Constraint(
          source,
          {orientation.uuid(), position.uuid(), velocity.uuid(),
           gyrobias.uuid(), accelbias.uuid()}),  // NOLINT(whitespace/braces)
      mean_(mean),
      sqrt_information_(covariance.inverse().llt().matrixU()) {}

void AbsoluteImuState3DStampedConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  orientation variable: " << variables().at(0) << "\n"
         << "  position variable: " << variables().at(1) << "\n"
         << "  velocity variable: " << variables().at(2) << "\n"
         << "  gyrobias variable: " << variables().at(3) << "\n"
         << "  accelbias variable:: " << variables().at(4) << "\n"
         << "  mean: " << mean().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";

  if (loss()) {
    stream << "  loss: ";
    loss()->print(stream);
  }
}

ceres::CostFunction* AbsoluteImuState3DStampedConstraint::costFunction() const {
  return new ceres::AutoDiffCostFunction<NormalPriorImuState3DCostFunctor, 15,
                                         4, 3, 3, 3, 3>(
      new NormalPriorImuState3DCostFunctor(sqrt_information_, mean_));
}

}}  // namespace beam_constraints::global

BOOST_CLASS_EXPORT_IMPLEMENT(
    beam_constraints::global::AbsoluteImuState3DStampedConstraint);
PLUGINLIB_EXPORT_CLASS(
    beam_constraints::global::AbsoluteImuState3DStampedConstraint,
    fuse_core::Constraint);
