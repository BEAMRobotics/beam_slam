#pragma once

#include <beam_constraints/frame_to_frame/frame_to_frame_transaction_base.h>

namespace beam_constraints { namespace frame_to_frame {

using Pose3DStampedTransaction = FrameToFrameTransactionBase<
    fuse_constraints::RelativePose3DStampedConstraint,
    fuse_constraints::AbsolutePose3DStampedConstraint>;

}} // namespace beam_constraints::frame_to_frame