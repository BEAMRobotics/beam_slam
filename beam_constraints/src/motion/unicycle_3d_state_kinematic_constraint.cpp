#include <beam_constraints/motion/unicycle_3d_state_kinematic_constraint.h>

#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>
#include <pluginlib/class_list_macros.h>

#include <beam_constraints/motion/unicycle_3d_state_cost_functor.h>

namespace beam_constraints { namespace motion {

Unicycle3DStateKinematicConstraint::Unicycle3DStateKinematicConstraint(
    const std::string& source,
    const fuse_variables::Position3DStamped& position1,
    const fuse_variables::Orientation3DStamped& orientation1,
    const fuse_variables::VelocityLinear3DStamped& linear_velocity1,
    const fuse_variables::VelocityAngular3DStamped& angular_velocity1,
    const fuse_variables::AccelerationLinear3DStamped& linear_acceleration1,
    const fuse_variables::Position3DStamped& position2,
    const fuse_variables::Orientation3DStamped& orientation2,
    const fuse_variables::VelocityLinear3DStamped& linear_velocity2,
    const fuse_variables::VelocityAngular3DStamped& angular_velocity2,
    const fuse_variables::AccelerationLinear3DStamped& linear_acceleration2,
    const fuse_core::Matrix15d& covariance)
    : fuse_core::Constraint(
          source,
          {position1.uuid(), orientation1.uuid(), linear_velocity1.uuid(),
           angular_velocity1.uuid(), linear_acceleration1.uuid(),
           position2.uuid(), orientation2.uuid(), linear_velocity2.uuid(),
           angular_velocity2.uuid(), linear_acceleration2.uuid()}), // NOLINT
      dt_((position2.stamp() - position1.stamp()).toSec()),
      sqrt_information_(covariance.inverse().llt().matrixU()) {}

void Unicycle3DStateKinematicConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  position variable 1: " << variables().at(0) << "\n"
         << "  orientation variable 1: " << variables().at(1) << "\n"
         << "  linear velocity variable 1: " << variables().at(2) << "\n"
         << "  angular velocity variable 1: " << variables().at(3) << "\n"
         << "  linear acceleration variable 1: " << variables().at(4) << "\n"
         << "  position variable 2: " << variables().at(5) << "\n"
         << "  orientation variable 2: " << variables().at(6) << "\n"
         << "  linear velocity variable 2: " << variables().at(7) << "\n"
         << "  angular velocity variable 2: " << variables().at(8) << "\n"
         << "  linear acceleration variable 2: " << variables().at(9) << "\n"
         << "  dt: " << dt() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";
}
/*
    position1. - 2
    yaw1. - 1
    linear_velocity1. - 2
    yaw_velocity1. - 1
    linear_acceleration1. - 2
    position2. - 2
    yaw2. - 1
    linear_velocity2. - 2
    yaw_velocity2. - 1
    linear_acceleration2 -2


position1. - 3
orientation1. - 3
linear_velocity1. - 3
angular_velocity1. - 3
linear_acceleration1. - 3
position2. - 3
orientation2. - 3
linear_velocity2. - 3
angular_velocity2. - 3
linear_acceleration2 -3*/

ceres::CostFunction* Unicycle3DStateKinematicConstraint::costFunction() const {
  return new ceres::AutoDiffCostFunction<Unicycle3DStateCostFunctor, 15, 3, 4,
                                         3, 3, 3, 3, 4, 3, 3, 3>(
      new Unicycle3DStateCostFunctor(dt_, sqrt_information_));
}

}} // namespace beam_constraints::motion

BOOST_CLASS_EXPORT_IMPLEMENT(
    beam_constraints::motion::Unicycle3DStateKinematicConstraint);
PLUGINLIB_EXPORT_CLASS(beam_constraints::motion::Unicycle3DStateKinematicConstraint,
                       fuse_core::Constraint);
