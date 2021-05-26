#pragma once

#include <beam_constraints/frame_to_frame/frame_to_frame_transaction_base.h>

namespace beam_constraints { namespace frame_to_frame {

using ImuState3DStampedTransaction = FrameToFrameTransactionBase<
    beam_constraints::frame_to_frame::RelativeImuState3DStampedConstraint,
    beam_constraints::global::AbsoluteImuState3DStampedConstraint>;

}} // namespace beam_constraints::frame_to_frame