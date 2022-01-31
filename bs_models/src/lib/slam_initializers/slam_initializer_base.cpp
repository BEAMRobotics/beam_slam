#include <bs_models/slam_initializers/slam_initializer_base.h>

#include <beam_utils/math.h>
#include <bs_common/bs_msgs.h>

namespace bs_models {

SLAMInitializerBase::SLAMInitializerBase() : fuse_core::AsyncSensorModel(1) {
  // advertise init path publisher
  results_publisher_ =
      private_node_handle_.advertise<bs_common::InitializedPathMsg>(
          "/local_mapper/slam_init/result", 10);
}

void SLAMInitializerBase::PublishResults() {
  bs_common::InitializedPathMsg msg;

  for (uint32_t i = 0; i < trajectory_.size(); i++) {
    Eigen::Matrix4d T = trajectory_[i];

    std_msgs::Header header;
    header.frame_id = extrinsics_.GetBaselinkFrameId();
    header.seq = i;
    header.stamp = times_[i];

    geometry_msgs::Point position;
    position.x = T(0, 3);
    position.y = T(1, 3);
    position.z = T(2, 3);

    Eigen::Matrix3d R = T.block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);

    geometry_msgs::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();

    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose.position = position;
    pose.pose.orientation = orientation;
    msg.poses.push_back(pose);
  }
  results_publisher_.publish(msg);
}

} // namespace bs_models
