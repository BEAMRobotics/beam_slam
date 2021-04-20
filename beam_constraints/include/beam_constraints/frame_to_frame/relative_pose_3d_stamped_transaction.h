#pragma once

#include <beam_constraints/frame_to_frame/frame_to_frame_transaction_base.h>

namespace beam_constraints { namespace frame_to_frame {

using RelativePose3DStampedTransaction = FrameToFrameTransactionBase<
    fuse_constraints::RelativePose3DStampedConstraint,
    fuse_constraints::AbsolutePose3DStampedConstraint>;

using RelativePoseWithVelocity3DStampedTransaction = FrameToFrameTransactionBase<
    fuse_constraints::RelativePoseWithVelocity3DStampedConstraint,
    fuse_constraints::AbsolutePoseWithVelocity3DStampedConstraint>;

}} // namespace beam_constraints::frame_to_frame