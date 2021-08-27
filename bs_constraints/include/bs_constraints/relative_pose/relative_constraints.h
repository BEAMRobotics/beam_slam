#pragma once

#include <fuse_constraints/relative_constraint.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

#include <bs_variables/accel_bias_3d_stamped.h>
#include <bs_variables/gyro_bias_3d_stamped.h>

namespace bs_constraints {
namespace relative_pose {

// Define unique names for the different variations of the relative constraint
using RelativeVelocityLinear3DStampedConstraint =
    fuse_constraints::RelativeConstraint<
        fuse_variables::VelocityLinear3DStamped>;
using RelativeGyroBias3DStampedConstraint =
    fuse_constraints::RelativeConstraint<bs_variables::GyroscopeBias3DStamped>;
using RelativeAccelBias3DStampedConstraint =
    fuse_constraints::RelativeConstraint<
        bs_variables::AccelerationBias3DStamped>;

}  // namespace relative_pose
}  // namespace bs_constraints

// Include the template implementation
#include <bs_constraints/relative_pose/relative_constraint_impl.h>

BOOST_CLASS_EXPORT_KEY(
    bs_constraints::relative_pose::RelativeVelocityLinear3DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(
    bs_constraints::relative_pose::RelativeGyroBias3DStampedConstraint);
BOOST_CLASS_EXPORT_KEY(
    bs_constraints::relative_pose::RelativeAccelBias3DStampedConstraint);
