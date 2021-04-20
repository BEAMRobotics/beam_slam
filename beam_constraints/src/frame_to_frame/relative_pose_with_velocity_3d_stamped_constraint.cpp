#include <beam_constraints/frame_to_frame/relative_pose_with_velocity_3d_stamped_constraint.h>
#include <beam_constraints/frame_to_frame/normal_delta_pose_with_velocity_3d_cost_functor.h>

#include <string>

#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>
#include <pluginlib/class_list_macros.h>

namespace beam_constraints { namespace frame_to_frame {

RelativePoseWithVelocity3DStampedConstraint::
    RelativePoseWithVelocity3DStampedConstraint(
        const std::string& source,
        const fuse_variables::Position3DStamped& position1,
        const fuse_variables::VelocityLinear3DStamped& velocity1,
        const fuse_variables::Orientation3DStamped& orientation1,
        const fuse_variables::Position3DStamped& position2,
        const fuse_variables::VelocityLinear3DStamped& velocity2,
        const fuse_variables::Orientation3DStamped& orientation2,
        const Eigen::Matrix<double, 10, 1>& delta,
        const fuse_core::Matrix9d& covariance)
    : fuse_core::Constraint(
          source, {position1.uuid(), velocity1.uuid(), orientation1.uuid(),
                   position2.uuid(), velocity2.uuid(),
                   orientation2.uuid()}),  // NOLINT(whitespace/braces)
      delta_(delta),
      sqrt_information_(covariance.inverse().llt().matrixU()) {}

void RelativePoseWithVelocity3DStampedConstraint::print(
    std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  position1 variable: " << variables().at(0) << "\n"
         << "  velocity1 variable: " << variables().at(1) << "\n"
         << "  orientation1 variable: " << variables().at(2) << "\n"
         << "  position2 variable: " << variables().at(3) << "\n"
         << "  velocity2 variable: " << variables().at(4) << "\n"
         << "  orientation2 variable: " << variables().at(5) << "\n"
         << "  delta: " << delta().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";
}

ceres::CostFunction* RelativePoseWithVelocity3DStampedConstraint::costFunction()
    const {
  return new ceres::AutoDiffCostFunction<
      NormalDeltaPoseWithVelocity3DCostFunctor, 9, 3, 3, 4, 3, 3, 4>(
      new NormalDeltaPoseWithVelocity3DCostFunctor(sqrt_information_, delta_));
}

}}	// beam_constraints::frame_to_frame

BOOST_CLASS_EXPORT_IMPLEMENT(beam_constraints::frame_to_frame::
                                 RelativePoseWithVelocity3DStampedConstraint);
PLUGINLIB_EXPORT_CLASS(beam_constraints::frame_to_frame::
                           RelativePoseWithVelocity3DStampedConstraint,
                       fuse_core::Constraint);
