#include <beam_models/frame_initializers/internal_frame_initializer.h>

#include <beam_common/utils.h>

namespace beam_models {
namespace frame_initializers {

InternalFrameInitializer& InternalFrameInitializer::GetInstance(
    int64_t poses_buffer_time) {
  static InternalFrameInitializer instance(poses_buffer_time);
  return instance;
}

InternalFrameInitializer::InternalFrameInitializer(int64_t poses_buffer_time)
    : poses_buffer_time_(poses_buffer_time) {}

bool InternalFrameInitializer::AddPose(const Eigen::Matrix4d& T_WORLD_SENSOR,
                                       const ros::Time& stamp,
                                       const std::string& sensor_frame_id) {
  // set poses
  if (poses_) {
    poses_ =
        std::make_shared<tf2::BufferCore>(ros::Duration(poses_buffer_time_));
    pose_lookup_ = std::make_shared<beam_common::PoseLookup>(poses_);
  }

  Eigen::Matrix4d T_SENSOR_BASELINK;
  bool lookup_success{true};
  if (sensor_frame_id == extrinsics_.GetBaselinkFrameId()) {
    T_SENSOR_BASELINK = Eigen::Matrix4d::Identity();
  } else if (sensor_frame_id == extrinsics_.GetImuFrameId()) {
    lookup_success = extrinsics_.GetT_IMU_BASELINK(T_SENSOR_BASELINK, stamp);
  } else if (sensor_frame_id == extrinsics_.GetCameraFrameId()) {
    lookup_success = extrinsics_.GetT_CAMERA_BASELINK(T_SENSOR_BASELINK, stamp);
  } else if (sensor_frame_id == extrinsics_.GetLidarFrameId()) {
    lookup_success = extrinsics_.GetT_LIDAR_BASELINK(T_SENSOR_BASELINK, stamp);
  } else {
    BEAM_ERROR(
        "Invalid sensor frame id provided to InternalFrameInitializer. Not "
        "adding pose.");
    return false;
  }

  if (!lookup_success) {
    return false;
  }

  Eigen::Matrix4d T_WORLD_BASELINK = T_WORLD_SENSOR * T_SENSOR_BASELINK;

  // build tf message and populate in buffer core
  geometry_msgs::TransformStamped tf_stamped;
  beam_common::EigenTransformToTransformStampedMsg(
      T_WORLD_BASELINK, stamp, 0, extrinsics_.GetWorldFrameId(),
      extrinsics_.GetBaselinkFrameId(), tf_stamped);

  std::string authority{"internal"};
  return poses_->setTransform(tf_stamped, authority, false);
}

}  // namespace frame_initializers
}  // namespace beam_models