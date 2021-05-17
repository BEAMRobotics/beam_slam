#pragma once

#include <ceres/normal_prior.h>

#include <beam_variables/imu_bias_gyro_3d_stamped.h>
#include <beam_variables/imu_bias_accel_3d_stamped.h>

namespace beam_constraints { namespace global {

template <class Variable>
AbsoluteConstraint3D<Variable>::AbsoluteConstraint3D(
    const std::string& source, const Variable& variable,
    const fuse_core::VectorXd& mean, const fuse_core::MatrixXd& covariance)
    : fuse_core::Constraint(source,
                            {variable.uuid()}), // NOLINT(whitespace/braces)
      mean_(mean),
      sqrt_information_(covariance.inverse().llt().matrixU()) {
  assert(mean.rows() == static_cast<int>(variable.size()));
  assert(covariance.rows() == static_cast<int>(variable.size()));
  assert(covariance.cols() == static_cast<int>(variable.size()));
}

template <class Variable>
AbsoluteConstraint3D<Variable>::AbsoluteConstraint3D(
    const std::string& source, const Variable& variable,
    const fuse_core::VectorXd& partial_mean,
    const fuse_core::MatrixXd& partial_covariance,
    const std::vector<size_t>& indices)
    : fuse_core::Constraint(source,
                            {variable.uuid()}) // NOLINT(whitespace/braces)
{
  assert(partial_mean.rows() == static_cast<int>(indices.size()));
  assert(partial_covariance.rows() == static_cast<int>(indices.size()));
  assert(partial_covariance.cols() == static_cast<int>(indices.size()));
  // Compute the sqrt information of the provided cov matrix
  fuse_core::MatrixXd partial_sqrt_information =
      partial_covariance.inverse().llt().matrixU();
  // Assemble a mean vector and sqrt information matrix from the provided
  // values, but in proper Variable order What are we doing here? The constraint
  // equation is defined as: cost(x) = ||A * (x - b)||^2 If we are measuring a
  // subset of dimensions, we only want to produce costs for the measured
  // dimensions. But the variable vectors will be full sized. We can make this
  // all work out by creating a non-square A matrix, where each row computes a
  // cost for one measured dimensions, and the columns are in the order defined
  // by the variable.
  mean_ = fuse_core::VectorXd::Zero(variable.size());
  sqrt_information_ =
      fuse_core::MatrixXd::Zero(indices.size(), variable.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    mean_(indices[i]) = partial_mean(i);
    sqrt_information_.col(indices[i]) = partial_sqrt_information.col(i);
  }
}

template <class Variable>
fuse_core::MatrixXd AbsoluteConstraint3D<Variable>::covariance() const {
  // We want to compute:
  // cov = (sqrt_info' * sqrt_info)^-1
  // With some linear algebra, we can swap the transpose and the inverse.
  // cov = (sqrt_info^-1) * (sqrt_info^-1)'
  // But sqrt_info _may_ not be square. So we need to compute the pseudoinverse
  // instead. Eigen doesn't have a pseudoinverse function (for probably very
  // legitimate reasons). So we set the right hand side to identity, then solve
  // using one of Eigen's many decompositions.
  auto I = fuse_core::MatrixXd::Identity(sqrt_information_.rows(),
                                         sqrt_information_.cols());
  fuse_core::MatrixXd pinv = sqrt_information_.colPivHouseholderQr().solve(I);
  return pinv * pinv.transpose();
}

template <class Variable>
void AbsoluteConstraint3D<Variable>::print(std::ostream& stream) const {
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  variable: " << variables().at(0) << "\n"
         << "  mean: " << mean().transpose() << "\n"
         << "  sqrt_info: " << sqrtInformation() << "\n";
}

template <class Variable>
ceres::CostFunction* AbsoluteConstraint3D<Variable>::costFunction() const {
  // Ceres ships with a "prior" cost function. Just use that here.
  return new ceres::NormalPrior(sqrt_information_, mean_);
}

template <>
inline std::string
    AbsoluteConstraint3D<fuse_variables::VelocityAngular3DStamped>::type()
        const {
  return "fuse_constraints::AbsoluteVelocityAngular3DStampedConstraint";
}

template <>
inline std::string
    AbsoluteConstraint3D<fuse_variables::VelocityLinear3DStamped>::type()
        const {
  return "fuse_constraints::AbsoluteVelocityLinear3DStampedConstraint";
}

template <>
inline std::string
    AbsoluteConstraint3D<fuse_variables::AccelerationLinear3DStamped>::type()
        const {
  return "fuse_constraints::AbsoluteAccelerationLinear3DStampedConstraint";
}

template <>
inline std::string
    AbsoluteConstraint3D<beam_variables::ImuBiasGyro3DStamped>::type()
        const {
  return "fuse_constraints::AbsoluteImuBiasGyro3DStampedConstraint";
}

template <>
inline std::string
    AbsoluteConstraint3D<beam_variables::ImuBiasAccel3DStamped>::type()
        const {
  return "fuse_constraints::AbsoluteImuBiasAccel3DStampedConstraint";
}

}} // namespace beam_constraints::global
