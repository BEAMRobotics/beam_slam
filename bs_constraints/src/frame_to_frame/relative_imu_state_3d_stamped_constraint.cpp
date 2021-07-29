#include <bs_constraints/frame_to_frame/relative_imu_state_3d_stamped_constraint.h>
#include <bs_constraints/frame_to_frame/normal_delta_imu_state_3d_cost_functor.h>

#include <string>

#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>
#include <pluginlib/class_list_macros.h>

namespace bs_constraints { namespace frame_to_frame {

RelativeImuState3DStampedConstraint::RelativeImuState3DStampedConstraint(
    const std::string& source,
    const fuse_variables::Orientation3DStamped& orientation1,
    const fuse_variables::Position3DStamped& position1,
    const fuse_variables::VelocityLinear3DStamped& velocity1,
    const bs_variables::GyroscopeBias3DStamped& gyrobias1,
    const bs_variables::AccelerationBias3DStamped& accelbias1,
    const fuse_variables::Orientation3DStamped& orientation2,
    const fuse_variables::Position3DStamped& position2,
    const fuse_variables::VelocityLinear3DStamped& velocity2,
    const bs_variables::GyroscopeBias3DStamped& gyrobias2,
    const bs_variables::AccelerationBias3DStamped& accelbias2,
    const Eigen::Matrix<double, 16, 1>& delta,
    const Eigen::Matrix<double, 15, 15>& covariance,
    const std::shared_ptr<bs_common::PreIntegrator> pre_integrator)
    : fuse_core::Constraint(
          source, {orientation1.uuid(), position1.uuid(), velocity1.uuid(),
                   gyrobias1.uuid(), accelbias1.uuid(), orientation2.uuid(),
                   position2.uuid(), velocity2.uuid(), gyrobias2.uuid(),
                   accelbias2.uuid()}),  // NOLINT(whitespace/braces)
      delta_(delta),
      sqrt_information_(covariance.inverse().llt().matrixU()) {}

void RelativeImuState3DStampedConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  orientation1 variable: " << variables().at(0) << "\n"
         << "  position1 variable: " << variables().at(1) << "\n"
         << "  velocity1 variable: " << variables().at(2) << "\n"
         << "  gyrobias1 variable: " << variables().at(3) << "\n"
         << "  accelbias1 variable: " << variables().at(4) << "\n"
         << "  orientation2 variable: " << variables().at(5) << "\n"
         << "  position2 variable: " << variables().at(6) << "\n"
         << "  velocity2 variable: " << variables().at(7) << "\n"
         << "  gyrobias2 variable: " << variables().at(8) << "\n"
         << "  accelbias2 variable: " << variables().at(9) << "\n"
         << "  delta: " << delta().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";
}

ceres::CostFunction* RelativeImuState3DStampedConstraint::costFunction() const {
  return new ceres::AutoDiffCostFunction<NormalDeltaImuState3DCostFunctor, 15,
                                         4, 3, 3, 3, 3, 4, 3, 3, 3, 3>(
      new NormalDeltaImuState3DCostFunctor(sqrt_information_, delta_));
}

}}  // namespace bs_constraints::frame_to_frame

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::frame_to_frame::RelativeImuState3DStampedConstraint);
PLUGINLIB_EXPORT_CLASS(
    bs_constraints::frame_to_frame::RelativeImuState3DStampedConstraint,
    fuse_core::Constraint);
