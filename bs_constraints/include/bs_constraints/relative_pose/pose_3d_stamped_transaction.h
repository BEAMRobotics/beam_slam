#pragma once

#include <bs_constraints/relative_pose/relative_pose_transaction_base.h>

namespace bs_constraints { namespace relative_pose {

using Pose3DStampedTransaction = RelativePoseTransactionBase<
    fuse_constraints::RelativePose3DStampedConstraint,
    fuse_constraints::AbsolutePose3DStampedConstraint>;

}} // namespace bs_constraints::relative_pose