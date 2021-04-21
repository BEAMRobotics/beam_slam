#pragma once

#include <beam_constraints/frame_to_frame/frame_to_frame_transaction_base.h>
#include <beam_constraints/frame_to_frame/relative_pose_with_velocity_3d_stamped_constraint.h>
#include <beam_constraints/global/absolute_pose_with_velocity_3d_stamped_constraint.h>

namespace beam_constraints { namespace frame_to_frame {

using Pose3DStampedTransaction = FrameToFrameTransactionBase<
    fuse_constraints::RelativePose3DStampedConstraint,
    fuse_constraints::AbsolutePose3DStampedConstraint>;

using PoseWithVelocity3DStampedTransaction = FrameToFrameTransactionBase<
    beam_constraints::frame_to_frame::RelativePoseWithVelocity3DStampedConstraint,
    beam_constraints::global::AbsolutePoseWithVelocity3DStampedConstraint>;

}} // namespace beam_constraints::frame_to_frame