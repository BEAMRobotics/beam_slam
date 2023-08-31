#include <bs_constraints/inertial/relative_imu_state_3d_stamped_constraint.h>

#include <string>

#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>
#include <pluginlib/class_list_macros.h>

#include <bs_constraints/inertial/normal_delta_imu_state_3d_cost_functor.h>

namespace bs_constraints {

RelativeImuState3DStampedConstraint::RelativeImuState3DStampedConstraint(
    const std::string& source, const bs_common::ImuState& imu_state_i,
    const bs_common::ImuState& imu_state_j,
    const std::shared_ptr<bs_common::PreIntegrator>& pre_integrator,
    const double info_weight)
    : fuse_core::Constraint(
          source,
          {imu_state_i.Orientation().uuid(), imu_state_i.Position().uuid(),
           imu_state_i.Velocity().uuid(), imu_state_i.GyroBias().uuid(),
           imu_state_i.AccelBias().uuid(), imu_state_j.Orientation().uuid(),
           imu_state_j.Position().uuid(), imu_state_j.Velocity().uuid(),
           imu_state_j.GyroBias().uuid(), imu_state_j.AccelBias().uuid()}),
      imu_state_i_(imu_state_i),
      imu_state_j_(imu_state_j),
      pre_integrator_(pre_integrator),
      info_weight_(info_weight) {}

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
         << "  ImuState_j AccelBias variable: " << variables().at(9) << "\n"
         << "  sqrt_info: \n"
         << info_weight_ * pre_integrator_->delta.sqrt_inv_cov << "\n";
}

ceres::CostFunction* RelativeImuState3DStampedConstraint::costFunction() const {
  return new ceres::AutoDiffCostFunction<NormalDeltaImuState3DCostFunctor, 15,
                                         4, 3, 3, 3, 3, 4, 3, 3, 3, 3>(
      new NormalDeltaImuState3DCostFunctor(imu_state_i_, pre_integrator_,
                                           info_weight_));
}

Eigen::Matrix4d RelativeImuState3DStampedConstraint::getRelativePose() const {
  Eigen::Matrix4d T_W_S1 = Eigen::Matrix4d::Identity();
  T_W_S1.block(0, 0, 3, 3) = imu_state_i_.OrientationMat();
  T_W_S1.block(0, 3, 3, 1) = imu_state_i_.PositionVec();

  Eigen::Matrix4d T_W_S2 = Eigen::Matrix4d::Identity();
  T_W_S2.block(0, 0, 3, 3) = imu_state_j_.OrientationMat();
  T_W_S2.block(0, 3, 3, 1) = imu_state_j_.PositionVec();

  Eigen::Matrix4d T_S1_S2 = beam::InvertTransform(T_W_S1) * T_W_S2;
  return T_S1_S2;
}

} // namespace bs_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::RelativeImuState3DStampedConstraint);
PLUGINLIB_EXPORT_CLASS(bs_constraints::RelativeImuState3DStampedConstraint,
                       fuse_core::Constraint);
