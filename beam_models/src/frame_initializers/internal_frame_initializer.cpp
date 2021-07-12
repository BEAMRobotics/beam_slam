#include <beam_models/frame_initializers/internal_frame_initializer.h>

namespace beam_models {
namespace frame_initializers {

InternalFrameInitializer::InternalFrameInitializer(
    int64_t poses_buffer_time, const std::string& sensor_frame_id)
    : FrameInitializerBase(sensor_frame_id) {
  poses_ = std::make_shared<tf2::BufferCore>(ros::Duration(poses_buffer_time));
  pose_lookup_.SetPoses(poses_);
}

bool InternalFrameInitializer::AddPose(const Eigen::Matrix4d& T_WORLD_SENSOR,
                                       const ros::Time& stamp,
                                       std::string sensor_frame_id) {
  if (sensor_frame_id.empty()) sensor_frame_id = sensor_frame_id_;

  // convert matrix to position and quaternion
  Eigen::Quaterniond orientation;
  Eigen::Vector3d position;
  beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_SENSOR, orientation,
                                                  position);

  // build tf message and populate in buffer core
  geometry_msgs::TransformStamped tf_stamped;
  tf_stamped.header.stamp = stamp;
  tf_stamped.header.frame_id = pose_lookup_.GetWorldFrameID();
  tf_stamped.child_frame_id = sensor_frame_id;
  tf_stamped.transform.translation.x = position[0];
  tf_stamped.transform.translation.y = position[1];
  tf_stamped.transform.translation.z = position[2];
  tf_stamped.transform.rotation.w = orientation.w();
  tf_stamped.transform.rotation.x = orientation.x();
  tf_stamped.transform.rotation.y = orientation.y();
  tf_stamped.transform.rotation.z = orientation.z();
  std::string authority{"internal"};
  return poses_->setTransform(tf_stamped, authority, false);
}

}  // namespace frame_initializers
}  // namespace beam_models