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
    : fuse_core::Constraint(source, {qwxyz_World_Imu.uuid()}),
      sqrt_information_(covariance.inverse().llt().matrixU()) {
  // q_inv = [qw, -qx, -qy, -qz]
  qwxyz_Imu_World_[0] = qwxyz_World_Imu.w();
  qwxyz_Imu_World_[1] = -qwxyz_World_Imu.x();
  qwxyz_Imu_World_[2] = -qwxyz_World_Imu.y();
  qwxyz_Imu_World_[3] = -qwxyz_World_Imu.z();
}

void GravityAlignmentStampedConstraint::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  Orientation variable: " << variables().at(0) << "\n"
         << "  qwxyz_Imu_World_: [" << qwxyz_Imu_World_[0] << ", "
         << qwxyz_Imu_World_[1] << ", " << qwxyz_Imu_World_[2] << ", "
         << qwxyz_Imu_World_[3] << "]\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";

  if (loss()) {
    stream << "  loss: ";
    loss()->print(stream);
  }
}

ceres::CostFunction* GravityAlignmentStampedConstraint::costFunction() const {
  return new ceres::AutoDiffCostFunction<GravityAlignmentCostFunctor, 2, 4>(
      new GravityAlignmentCostFunctor(sqrt_information_, qwxyz_Imu_World_));
}

}} // namespace bs_constraints::global

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::global::GravityAlignmentStampedConstraint);
PLUGINLIB_EXPORT_CLASS(
    bs_constraints::global::GravityAlignmentStampedConstraint,
    fuse_core::Constraint);
