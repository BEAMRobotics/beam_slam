#include <beam_models/frame_initializers/pose_file_frame_initializer.h>

#include <boost/filesystem.hpp>

#include <beam_mapping/Poses.h>
#include <beam_utils/log.h>

#include <beam_common/utils.h>

namespace beam_models {
namespace frame_initializers {

PoseFileFrameInitializer::PoseFileFrameInitializer(
    const std::string& file_path) {
  if (!boost::filesystem::exists(file_path)) {
    BEAM_ERROR("Pose file not found: {}", file_path);
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

  // check for valid frame ids
  if (poses_reader.GetFixedFrame() != extrinsics_.GetWorldFrameId()) {
    BEAM_WARN(
        "Fixed frame id in pose file is not consistend with world frame id "
        "from extrinsics. Using world frame from from extrinsics.");
  }

  if (poses_reader.GetMovingFrame() != extrinsics_.GetImuFrameId() &&
      poses_reader.GetMovingFrame() != extrinsics_.GetCameraFrameId() &&
      poses_reader.GetMovingFrame() != extrinsics_.GetLidarFrameId()) {
    BEAM_ERROR(
        "Moving frame id in pose file must be equal to one of the frame ids in "
        "the extrinsics.");
    throw std::invalid_argument{"Invalid moving frame id."};
  }

  if (!extrinsics_.IsStatic() &&
      poses_reader.GetMovingFrame() != extrinsics_.GetBaselinkFrameId()) {
    BEAM_ERROR(
        "Cannot use pose file with a moving frame that is not equal to the "
        "baselink frame when extrinsics are not static.");
    throw std::runtime_error{"Extrinsics must be static."};
  }

  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>
      transforms = poses_reader.GetPoses();
  std::vector<ros::Time> timestamps = poses_reader.GetTimeStamps();

  // create buffer core with cache time slightly larger than the difference
  // between the min and max timestamps. Also include a minimum
  int64_t cache_time =
      1.2 * (timestamps[timestamps.size() - 1].toSec() - timestamps[0].toSec());
  if (cache_time < 10) {
    cache_time = 10;
  }

  poses_ = std::make_shared<tf2::BufferCore>(ros::Duration(cache_time));
  pose_lookup_ = std::make_shared<beam_common::PoseLookup>(poses_);

  Eigen::Matrix4d T_BASELINK_MOVINGFRAME;
  if (!pose_lookup_->GetT_BASELINK_SENSOR(T_BASELINK_MOVINGFRAME,
                                          poses_reader.GetMovingFrame())) {
    throw std::runtime_error{""};  // additional warning thrown by
                                   // PoseLookup::GetT_BASELINK_SENSOR
  }

  Eigen::Matrix4d T_MOVINGFRAME_BASELINK =
      beam::InvertTransform(T_BASELINK_MOVINGFRAME);
  for (int i = 0; i < transforms.size(); i++) {
    const Eigen::Matrix4d& T_WORLD_MOVINGFRAME = transforms[i].matrix();
    Eigen::Matrix4d T_WORLD_BASELINK =
        T_WORLD_MOVINGFRAME * T_MOVINGFRAME_BASELINK;
    geometry_msgs::TransformStamped tf_stamped;
    beam_common::EigenTransformToTransformStampedMsg(
        T_WORLD_BASELINK, timestamps[i], i, extrinsics_.GetWorldFrameId(),
        extrinsics_.GetBaselinkFrameId(), tf_stamped);
    poses_->setTransform(tf_stamped, authority_, false);
  }
}

}  // namespace frame_initializers
}  // namespace beam_models