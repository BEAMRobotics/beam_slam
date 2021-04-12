#include <beam_models/frame_initializers/pose_file_frame_initializer.h>

#include <boost/filesystem.hpp>

#include <beam_mapping/Poses.h>
#include <beam_utils/log.h>

namespace beam_models { namespace frame_initializers {

PoseFileFrameInitializer::PoseFileFrameInitializer(
    const std::string& file_path, const std::string& sensor_frame_id,
    const std::string& baselink_frame_id, const std::string& world_frame_id,
    bool static_extrinsics) {
  // frame ids might change:
  std::string _sensor_frame_id = sensor_frame_id;
  std::string _baselink_frame_id = baselink_frame_id;
  std::string _world_frame_id = world_frame_id;

  if (!boost::filesystem::exists(file_path)) {
    ROS_ERROR("Pose file not found: %s", file_path.c_str());
    throw std::invalid_argument{"Pose file not found."};
  }
  beam_mapping::Poses poses_reader;
  std::string extension = boost::filesystem::extension(file_path);
  if (extension == "json") {
    poses_reader.LoadFromJSON(file_path);
  } else if (extension == "txt") {
    poses_reader.LoadFromTXT(file_path);
  } else if (extension == "ply") {
    poses_reader.LoadFromPLY(file_path);
  } else {
    ROS_ERROR("Invalid file extension for pose file. Options: json, txt, ply");
    throw std::invalid_argument{"Invalid extensions type."};
  }

  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>
      transforms = poses_reader.GetPoses();
  std::vector<ros::Time> timestamps = poses_reader.GetTimeStamps();

  std::shared_ptr<tf2::BufferCore> poses = std::make_shared<tf2::BufferCore>();

  if (_baselink_frame_id.empty()) {
    _baselink_frame_id = poses_reader.GetFixedFrame();
  } else if (_baselink_frame_id != poses_reader.GetFixedFrame()) {
    ROS_WARN("Baselink frame supplied to PoseFrameInitializer not consistent "
             "with pose file.");
  }

  if (_world_frame_id.empty()) {
    _world_frame_id = poses_reader.GetMovingFrame();
  } else if (_world_frame_id != poses_reader.GetMovingFrame()) {
    ROS_WARN("World frame supplied to PoseFrameInitializer not consistent "
             "with pose file.");
  }

  if (_sensor_frame_id.empty()) { _sensor_frame_id = _baselink_frame_id; }

  for (int i = 0; i < transforms.size(); i++) {
    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.header.stamp = timestamps[i];
    tf_stamped.header.seq = i;
    tf_stamped.header.frame_id = _world_frame_id;
    tf_stamped.child_frame_id = _baselink_frame_id;
    tf_stamped.transform.translation.x = transforms[i](0, 3);
    tf_stamped.transform.translation.y = transforms[i](1, 3);
    tf_stamped.transform.translation.z = transforms[i](2, 3);
    Eigen::Matrix3d R = transforms[i].rotation();
    Eigen::Quaterniond q(R);
    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();
    std::string authority{"poses_file"};
    poses->setTransform(tf_stamped, authority, false);
  }

  beam_common::PoseLookup::Params pose_lookup_params{
      .poses = poses,
      .sensor_frame = _sensor_frame_id,
      .baselink_frame = _baselink_frame_id,
      .world_frame = _world_frame_id,
      .static_extrinsics = static_extrinsics,};
  pose_lookup_ = std::make_unique<beam_common::PoseLookup>(pose_lookup_params);
}

bool PoseFileFrameInitializer::GetEstimatedPose(
    const ros::Time& time, Eigen::Matrix4d& T_WORLD_SENSOR) {
  if (pose_lookup_ == nullptr) { return false; }
  bool result = pose_lookup_->GetT_WORLD_SENSOR(T_WORLD_SENSOR, time);
  return result;
}

}} // namespace beam_models::frame_initializers