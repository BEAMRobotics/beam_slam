#include <beam_models/frame_initializers/pose_file_frame_initializer.h>

#include <boost/filesystem.hpp>

#include <beam_mapping/Poses.h>
#include <beam_utils/log.h>

namespace beam_models {
namespace frame_initializers {

PoseFileFrameInitializer::PoseFileFrameInitializer(
    const std::string& file_path, const std::string& sensor_frame_id)
    : sensor_frame_id_(sensor_frame_id) {
  if (!boost::filesystem::exists(file_path)) {
    ROS_ERROR("Pose file not found: %s", file_path.c_str());
    throw std::invalid_argument{"Pose file not found."};
  }

  beam_mapping::Poses poses_reader;
  std::string extension = boost::filesystem::extension(file_path);
  if (extension == ".json") {
    poses_reader.LoadFromJSON(file_path);
  } else if (extension == ".txt") {
    poses_reader.LoadFromTXT(file_path);
  } else if (extension == ".ply") {
    poses_reader.LoadFromPLY(file_path);
  } else {
    BEAM_ERROR(
        "Invalid file extension for pose file. Options: .json, .txt, .ply");
    throw std::invalid_argument{"Invalid extensions type."};
  }

  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>
      transforms = poses_reader.GetPoses();
  std::vector<ros::Time> timestamps = poses_reader.GetTimeStamps();

  if (pose_lookup_.GetBaselinkFrameID() != poses_reader.GetFixedFrame()) {
    BEAM_WARN(
        "Baselink frame supplied to PoseFrameInitializer is not consistent "
        "with pose file.");
  }

  if (pose_lookup_.GetWorldFrameID() != poses_reader.GetMovingFrame()) {
    BEAM_WARN(
        "World frame supplied to PoseFrameInitializer is not consistent "
        "with pose file.");
  }

  if (sensor_frame_id_.empty()) {
    sensor_frame_id_ = pose_lookup_.GetBaselinkFrameID();
  }

  // create buffer core with cache time slightly larger than the difference
  // between the min and max timestamps. Also include a minimum
  double cache_time =
      1.2 * (timestamps[timestamps.size() - 1].toSec() - timestamps[0].toSec());
  if (cache_time < 10) {
    cache_time = 10;
  }
  std::shared_ptr<tf2::BufferCore> poses =
      std::make_shared<tf2::BufferCore>(ros::Duration(cache_time));

  for (int i = 0; i < transforms.size(); i++) {
    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.header.stamp = timestamps[i];
    tf_stamped.header.seq = i;
    tf_stamped.header.frame_id = pose_lookup_.GetWorldFrameID();
    tf_stamped.child_frame_id = pose_lookup_.GetBaselinkFrameID();
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

  pose_lookup_.SetPoses(poses);
}

bool PoseFileFrameInitializer::GetEstimatedPose(
    const ros::Time& time, Eigen::Matrix4d& T_WORLD_SENSOR) {
  bool result =
      pose_lookup_.GetT_WORLD_SENSOR(T_WORLD_SENSOR, sensor_frame_id_, time);
  return result;
}

}  // namespace frame_initializers
}  // namespace beam_models