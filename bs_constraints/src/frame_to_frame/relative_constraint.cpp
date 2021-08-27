#include <bs_constraints/frame_to_frame/relative_constraint.h>

#include <pluginlib/class_list_macros.h>
#include <boost/serialization/export.hpp>

BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::frame_to_frame::RelativeVelocityLinear3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::frame_to_frame::RelativeGyroBias3DStampedConstraint);
BOOST_CLASS_EXPORT_IMPLEMENT(
    bs_constraints::frame_to_frame::RelativeAccelBias3DStampedConstraint);

PLUGINLIB_EXPORT_CLASS(
    bs_constraints::frame_to_frame::RelativeVelocityLinear3DStampedConstraint,
    fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
    bs_constraints::frame_to_frame::RelativeGyroBias3DStampedConstraint,
    fuse_core::Constraint);
PLUGINLIB_EXPORT_CLASS(
    bs_constraints::frame_to_frame::RelativeAccelBias3DStampedConstraint,
    fuse_core::Constraint);
