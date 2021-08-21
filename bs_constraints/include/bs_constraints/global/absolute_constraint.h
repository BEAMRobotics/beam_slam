#pragma once

#include <fuse_constraints/absolute_constraint.h>
#include <fuse_variables/acceleration_linear_3d_stamped.h>
#include <fuse_variables/velocity_angular_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

#include <bs_variables/accel_bias_3d_stamped.h>
#include <bs_variables/gyro_bias_3d_stamped.h>

namespace bs_constraints {
namespace global {

// Define unique names for the different variations of the absolute constraint
using AbsoluteVelocityAngular3DStampedConstraint =
    fuse_constraints::AbsoluteConstraint<
        fuse_variables::VelocityAngular3DStamped>;
using AbsoluteVelocityLinear3DStampedConstraint =
    fuse_constraints::AbsoluteConstraint<
        fuse_variables::VelocityLinear3DStamped>;
using AbsoluteAccelerationLinear3DStampedConstraint =
    fuse_constraints::AbsoluteConstraint<
        fuse_variables::AccelerationLinear3DStamped>;
using AbsoluteGyroBias3DStampedConstraint =
    fuse_constraints::AbsoluteConstraint<bs_variables::GyroscopeBias3DStamped>;
using AbsoluteAccelBias3DStampedConstraint =
    fuse_constraints::AbsoluteConstraint<
        bs_variables::AccelerationBias3DStamped>;

}  // namespace global
}  // namespace bs_constraints

// Include the template implementation
#include <bs_constraints/global/absolute_constraint_impl.h>

BOOST_CLASS_EXPORT_KEY(
    bs_constraints::global::AbsoluteVelocityAngular3DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(
    bs_constraints::global::AbsoluteVelocityLinear3DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(
    bs_constraints::global::AbsoluteAccelerationLinear3DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(
    bs_constraints::global::AbsoluteGyroBias3DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(
    bs_constraints::global::AbsoluteAccelBias3DStampedConstraint);