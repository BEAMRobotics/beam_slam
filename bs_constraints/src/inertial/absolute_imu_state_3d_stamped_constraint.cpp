#include <bs_constraints/inertial/absolute_imu_state_3d_stamped_constraint.h>
#include <bs_constraints/inertial/normal_prior_imu_state_3d_cost_functor.h>

#include <string>

#include <Eigen/Dense>
#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>
#include <pluginlib/class_list_macros.h>

namespace bs_constraints {

AbsoluteImuState3DStampedConstraint::AbsoluteImuState3DStampedConstraint(
    const std::string& source, const bs_common::ImuState& imu_state,
    const Eigen::Matrix<double, 16, 1>& mean,
    const Eigen::Matrix<double, 15, 15>& covariance)
    : fuse_core::Constraint(
          source, {imu_state.Orientation().uuid(), imu_state.Position().uuid(),
                   imu_state.Velocity().uuid(), imu_state.GyroBias().uuid(),
                   imu_state.AccelBias().uuid()}), // NOLINT(whitespace/braces)
      mean_(mean),
      sqrt_information_(covariance.inverse().llt().matrixU()) {}

void AbsoluteImuState3DStampedConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  ImuState Orientation variable: " << variables().at(0) << "\n"
         << "  ImuState Position variable: " << variables().at(1) << "\n"
         << "  ImuState Velocity variable: " << variables().at(2) << "\n"
         << "  ImuState Gyrobias variable: " << variables().at(3) << "\n"
         << "  ImuState Accelbias variable:: " << variables().at(4) << "\n"
         << "  mean: " << mean().transpose() << "\n"
         << "  sqrt_info: \n"
         << sqrtInformation() << "\n";

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

} // namespace bs_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::AbsoluteImuState3DStampedConstraint);
PLUGINLIB_EXPORT_CLASS(bs_constraints::AbsoluteImuState3DStampedConstraint,
                       fuse_core::Constraint);
