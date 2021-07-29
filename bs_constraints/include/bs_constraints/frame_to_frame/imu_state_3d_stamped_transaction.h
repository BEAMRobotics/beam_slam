#pragma once

#include <bs_constraints/frame_to_frame/frame_to_frame_transaction_base.h>

namespace bs_constraints { namespace frame_to_frame {

using ImuState3DStampedTransaction = FrameToFrameTransactionBase<
    bs_constraints::frame_to_frame::RelativeImuState3DStampedConstraint,
    bs_constraints::global::AbsoluteImuState3DStampedConstraint>;

}} // namespace bs_constraints::frame_to_frame