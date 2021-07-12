#include <beam_models/frame_initializers/internal_frame_initializer.h>

namespace beam_models {
namespace frame_initializers {

InternalFrameInitializer& InternalFrameInitializer::GetInstance() {
  static InternalFrameInitializer instance;
  return instance;
}

bool InternalFrameInitializer::GetEstimatedPose(const ros::Time& time,
                                                Eigen::Matrix4d& T_WORLD_SENSOR,
                                                std::string frame_id) {
  if (pose_lookup_ == nullptr) {
    return false;
  }
  // set to desired frame id
  pose_lookup_params_.sensor_frame = frame_id;
  bool result = pose_lookup_->GetT_WORLD_SENSOR(T_WORLD_SENSOR, time);
  return result;
}

InternalFrameInitializer::AddPose(const ros::Time& stamp,
                                  const Eigen::Matrix4d& T_WORLD_SENSOR,
                                  std::string frame_id) {
  if (poses_ == nullptr) {
    poses_ =
        std::make_shared<tf2::BufferCore>(ros::Duration(poses_buffer_time));
    pose_lookup_params_.poses = poses_;
  }
  if (pose_lookup_params_.baselink_frame.empty()) {
    pose_lookup_params_.baselink_frame = extrinsics->baselink();
  }
  if (pose_lookup_params_.world_frame.empty()) {
    pose_lookup_params_.world_frame = extrinsics->world_frame();
  }
  if (pose_lookup_ == nullptr) {
    pose_lookup_ =
        std::make_unique<beam_common::PoseLookup>(pose_lookup_params_);
  }

  // convert matrix to position and quaternion
  Eigen::Quaterniond orientation;
  Eigen::Vector3d position;
  beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_SENSOR, orientation,
                                                  position);
  // build tf message and populate in buffer core
  geometry_msgs::TransformStamped tf_stamped;
  tf_stamped.header.stamp = stamp;
  tf_stamped.header.frame_id = "/" + extrinsics->world_frame();
  tf_stamped.child_frame_id = frame_id;
  tf_stamped.transform.translation.x = position.x;
  tf_stamped.transform.translation.y = position.y;
  tf_stamped.transform.translation.z = position.z;
  tf_stamped.transform.rotation.x = orientation.x;
  tf_stamped.transform.rotation.y = orientation.y;
  tf_stamped.transform.rotation.z = orientation.z;
  tf_stamped.transform.rotation.w = orientation.w;
  std::string authority{"odometry"};
  poses_->setTransform(tf_stamped, authority, false);
}
}  // namespace frame_initializers
}  // namespace beam_models