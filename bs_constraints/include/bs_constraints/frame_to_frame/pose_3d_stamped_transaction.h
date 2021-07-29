#pragma once

#include <bs_constraints/frame_to_frame/frame_to_frame_transaction_base.h>

namespace bs_constraints { namespace frame_to_frame {

using Pose3DStampedTransaction = FrameToFrameTransactionBase<
    fuse_constraints::RelativePose3DStampedConstraint,
    fuse_constraints::AbsolutePose3DStampedConstraint>;

}} // namespace bs_constraints::frame_to_frame