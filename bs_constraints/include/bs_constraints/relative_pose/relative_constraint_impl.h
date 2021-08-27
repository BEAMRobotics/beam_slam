#pragma once

#include <fuse_constraints/relative_constraint_impl.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

#include <bs_variables/accel_bias_3d_stamped.h>
#include <bs_variables/gyro_bias_3d_stamped.h>

namespace fuse_constraints {

template <>
inline std::string fuse_constraints::RelativeConstraint<
    fuse_variables::VelocityLinear3DStamped>::type() const {
  return "fuse_constraints::RelativeVelocityLinear3DStampedConstraint";
}

template <>
inline std::string fuse_constraints::RelativeConstraint<
    bs_variables::GyroscopeBias3DStamped>::type() const {
  return "fuse_constraints::RelativeGyroBias3DStampedConstraint";
}

template <>
inline std::string fuse_constraints::RelativeConstraint<
    bs_variables::AccelerationBias3DStamped>::type() const {
  return "fuse_constraints::RelativeAccelBias3DStampedConstraint";
}

}  // namespace fuse_constraints
