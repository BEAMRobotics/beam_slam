#include <bs_constraints/relative_pose/relative_pose_3d_stamped_with_extrinsics_constraint.h>

#include <bs_constraints/relative_pose/delta_pose_3d_with_extrinsics_cost_functor.h>
#include <pluginlib/class_list_macros.h>

#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>

#include <string>

namespace bs_constraints {

RelativePose3DStampedWithExtrinsicsConstraint::
    RelativePose3DStampedWithExtrinsicsConstraint(
        const std::string& source,
        const fuse_variables::Position3DStamped& position1,
        const fuse_variables::Orientation3DStamped& orientation1,
        const fuse_variables::Position3DStamped& position2,
        const fuse_variables::Orientation3DStamped& orientation2,
        const bs_variables::Position3D& position_extrinsics,
        const bs_variables::Orientation3D& orientation_extrinsics,
        const fuse_core::Vector7d& d_Sensor1_Sensor2,
        const fuse_core::Matrix6d& covariance)
    : fuse_core::Constraint(
          source, {position1.uuid(), orientation1.uuid(), position2.uuid(),
                   orientation2.uuid(), position_extrinsics.uuid(),
                   orientation_extrinsics.uuid()}), // NOLINT(whitespace/braces)
      d_Sensor1_Sensor2_(d_Sensor1_Sensor2),
      sqrt_information_(covariance.inverse().llt().matrixU()) {}

void RelativePose3DStampedWithExtrinsicsConstraint::print(
    std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  position1 variable: " << variables().at(0) << "\n"
         << "  orientation1 variable: " << variables().at(1) << "\n"
         << "  position2 variable: " << variables().at(2) << "\n"
         << "  orientation2 variable: " << variables().at(3) << "\n"
         << "  extrinsics position variable: " << variables().at(4) << "\n"
         << "  extrinsics orientation variable: " << variables().at(5) << "\n"
         << "  delta: " << d_Sensor1_Sensor2_.transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";
}

ceres::CostFunction*
    RelativePose3DStampedWithExtrinsicsConstraint::costFunction() const {
  // 6 residuals and 3 sets of poses each with 3 translation variables and then
  // 4 rotation variables
  return new ceres::AutoDiffCostFunction<DeltaPose3DWithExtrinsicsCostFunctor,
                                         6, 3, 4, 3, 4, 3, 4>(
      new DeltaPose3DWithExtrinsicsCostFunctor(sqrt_information_,
                                               d_Sensor1_Sensor2_));
}

} // namespace bs_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::RelativePose3DStampedWithExtrinsicsConstraint);
PLUGINLIB_EXPORT_CLASS(
    bs_constraints::RelativePose3DStampedWithExtrinsicsConstraint,
    fuse_core::Constraint);
