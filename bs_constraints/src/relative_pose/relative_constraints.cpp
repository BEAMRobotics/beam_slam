#include <bs_constraints/relative_pose/relative_constraints.h>

#include <boost/serialization/export.hpp>
#include <pluginlib/class_list_macros.h>

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::RelativeVelocityLinear3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::RelativeGyroBias3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::RelativeAccelBias3DStampedConstraint);

PLUGINLIB_EXPORT_CLASS(
    bs_constraints::RelativeVelocityLinear3DStampedConstraint,
    fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(bs_constraints::RelativeGyroBias3DStampedConstraint,
                       fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(bs_constraints::RelativeAccelBias3DStampedConstraint,
                       fuse_core::Constraint);
