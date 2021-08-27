#pragma once

#include <bs_constraints/relative_pose/relative_pose_transaction_base.h>

namespace bs_constraints { namespace relative_pose {

using ImuState3DStampedTransaction = RelativePoseTransactionBase<
    bs_constraints::relative_pose::RelativeImuState3DStampedConstraint,
    bs_constraints::global::AbsoluteImuState3DStampedConstraint>;

}} // namespace bs_constraints::relative_pose