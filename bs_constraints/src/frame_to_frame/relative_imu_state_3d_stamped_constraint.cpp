#include <bs_constraints/frame_to_frame/relative_imu_state_3d_stamped_constraint.h>
#include <bs_constraints/frame_to_frame/delta_imu_state_3D_cost_function.h>

#include <string>

#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>
#include <pluginlib/class_list_macros.h>

namespace bs_constraints {
namespace frame_to_frame {

RelativeImuState3DStampedConstraint::RelativeImuState3DStampedConstraint(
    const std::string& source, const bs_common::ImuState& imu_state_i,
    const bs_common::ImuState& imu_state_j,
    const std::shared_ptr<bs_common::PreIntegrator> pre_integrator)
    : fuse_core::Constraint(
          source,
          {imu_state_i.Orientation().uuid(), imu_state_i.Position().uuid(),
           imu_state_i.Velocity().uuid(), imu_state_i.GyroBias().uuid(),
           imu_state_i.AccelBias().uuid(), imu_state_j.Orientation().uuid(),
           imu_state_j.Position().uuid(), imu_state_j.Velocity().uuid(),
           imu_state_j.GyroBias().uuid(), imu_state_j.AccelBias().uuid()}),
      imu_state_i_(imu_state_i),
      imu_state_j_(imu_state_j),
      pre_integrator_(*pre_integrator) {}

void RelativeImuState3DStampedConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  ImuState_i Orientation variable: " << variables().at(0) << "\n"
         << "  ImuState_i Position variable: " << variables().at(1) << "\n"
         << "  ImuState_i Velocity variable: " << variables().at(2) << "\n"
         << "  ImuState_i GyroBias variable: " << variables().at(3) << "\n"
         << "  ImuState_i AccelBias variable: " << variables().at(4) << "\n"
         << "  ImuState_j Orientation variable: " << variables().at(5) << "\n"
         << "  ImuState_j Position variable: " << variables().at(6) << "\n"
         << "  ImuState_j Velocity variable: " << variables().at(7) << "\n"
         << "  ImuState_j GyroBias variable: " << variables().at(8) << "\n"
         << "  ImuState_j AccelBias variable: " << variables().at(9) << "\n";
}

ceres::CostFunction* RelativeImuState3DStampedConstraint::costFunction() const {
  return new DeltaImuState3DCostFunction(imu_state_i_, imu_state_j_,
                                         pre_integrator_);
}

}  // namespace frame_to_frame
}  // namespace bs_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::frame_to_frame::RelativeImuState3DStampedConstraint);
PLUGINLIB_EXPORT_CLASS(
    bs_constraints::frame_to_frame::RelativeImuState3DStampedConstraint,
    fuse_core::Constraint);
