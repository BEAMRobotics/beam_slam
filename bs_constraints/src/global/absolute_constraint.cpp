#include <bs_constraints/global/absolute_constraint.h>

#include <boost/serialization/export.hpp>
#include <pluginlib/class_list_macros.h>

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::AbsoluteVelocityAngular3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::AbsoluteVelocityLinear3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::AbsoluteAccelerationLinear3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::AbsoluteGyroBias3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::AbsoluteAccelBias3DStampedConstraint);

PLUGINLIB_EXPORT_CLASS(
    bs_constraints::AbsoluteVelocityAngular3DStampedConstraint,
    fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
    bs_constraints::AbsoluteVelocityLinear3DStampedConstraint,
    fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
    bs_constraints::AbsoluteAccelerationLinear3DStampedConstraint,
    fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(bs_constraints::AbsoluteGyroBias3DStampedConstraint,
                       fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(bs_constraints::AbsoluteAccelBias3DStampedConstraint,
                       fuse_core::Constraint);
