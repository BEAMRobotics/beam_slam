#pragma once

#include <fuse_constraints/absolute_constraint_impl.h>
#include <fuse_variables/acceleration_linear_3d_stamped.h>
#include <fuse_variables/velocity_angular_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

#include <bs_variables/accel_bias_3d_stamped.h>
#include <bs_variables/gyro_bias_3d_stamped.h>

namespace fuse_constraints {

template <>
inline std::string fuse_constraints::AbsoluteConstraint<
    fuse_variables::VelocityAngular3DStamped>::type() const {
  return "fuse_constraints::AbsoluteVelocityAngular3DStampedConstraint";
}

template <>
inline std::string fuse_constraints::AbsoluteConstraint<
    fuse_variables::VelocityLinear3DStamped>::type() const {
  return "fuse_constraints::AbsoluteVelocityLinear3DStampedConstraint";
}

template <>
inline std::string fuse_constraints::AbsoluteConstraint<
    fuse_variables::AccelerationLinear3DStamped>::type() const {
  return "fuse_constraints::AbsoluteAccelerationLinear3DStampedConstraint";
}

template <>
inline std::string fuse_constraints::AbsoluteConstraint<
    bs_variables::GyroscopeBias3DStamped>::type() const {
  return "fuse_constraints::AbsoluteGyroBias3DStampedConstraint";
}

template <>
inline std::string fuse_constraints::AbsoluteConstraint<
    bs_variables::AccelerationBias3DStamped>::type() const {
  return "fuse_constraints::AbsoluteAccelBias3DStampedConstraint";
}

}  // namespace fuse_constraints
