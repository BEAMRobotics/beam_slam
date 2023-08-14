#include <bs_constraints/global/gravity_alignment_cost_functor.h>
#include <bs_constraints/global/gravity_alignment_stamped_constraint.h>

#include <Eigen/Dense>
#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>
#include <pluginlib/class_list_macros.h>

namespace bs_constraints { namespace global {

GravityAlignmentStampedConstraint::GravityAlignmentStampedConstraint(
    const std::string& source,
    const fuse_variables::Orientation3DStamped& qwxyz_World_Imu,
    const Eigen::Matrix<double, 2, 2>& covariance)
    : fuse_core::Constraint(source, {qwxyz_Imu_World_.uuid()}),
      sqrt_information_(covariance.inverse().llt().matrixU()) {
  auto qwxyz_Imu_World = qwxyz_World_Imu.inverse();
  qwxyz_Imu_World_[0] = qwxyz_Imu_World.w();
  qwxyz_Imu_World_[1] = qwxyz_Imu_World.x();
  qwxyz_Imu_World_[2] = qwxyz_Imu_World.y();
  qwxyz_Imu_World_[3] = qwxyz_Imu_World.z();
}

void GravityAlignmentStampedConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  Orientation variable: " << variables().at(0) << "\n"
         << "  ImuState Position variable: " << variables().at(1) << "\n"
         << "  mean: " << mean().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";

  if (loss()) {
    stream << "  loss: ";
    loss()->print(stream);
  }
}

ceres::CostFunction* GravityAlignmentStampedConstraint::costFunction() const {
  return new ceres::AutoDiffCostFunction<GravityAlignmentCostFunctor, 4, 4>(
      new GravityAlignmentCostFunctor(sqrt_information_, qwxyz_Imu_World_));
}

}} // namespace bs_constraints::global

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::global::GravityAlignmentStampedConstraint);
PLUGINLIB_EXPORT_CLASS(
    bs_constraints::global::GravityAlignmentStampedConstraint,
    fuse_core::Constraint);
