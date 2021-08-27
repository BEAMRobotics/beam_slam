#include <bs_constraints/relative_pose/relative_constraints.h>

#include <pluginlib/class_list_macros.h>
#include <boost/serialization/export.hpp>

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::relative_pose::RelativeVelocityLinear3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::relative_pose::RelativeGyroBias3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::relative_pose::RelativeAccelBias3DStampedConstraint);

PLUGINLIB_EXPORT_CLASS(
    bs_constraints::relative_pose::RelativeVelocityLinear3DStampedConstraint,
    fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
    bs_constraints::relative_pose::RelativeGyroBias3DStampedConstraint,
    fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
    bs_constraints::relative_pose::RelativeAccelBias3DStampedConstraint,
    fuse_core::Constraint);
