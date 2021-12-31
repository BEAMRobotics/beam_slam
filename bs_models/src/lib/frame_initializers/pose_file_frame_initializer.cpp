#include <bs_models/frame_initializers/pose_file_frame_initializer.h>

#include <boost/filesystem.hpp>

#include <beam_mapping/Poses.h>
#include <beam_utils/log.h>

#include <bs_common/utils.h>

namespace bs_models {
namespace frame_initializers {

PoseFileFrameInitializer::PoseFileFrameInitializer(
    const std::string& file_path) {
  authority_ = "poses_file";

  if (!boost::filesystem::exists(file_path)) {
    BEAM_ERROR("Pose file not found: {}", file_path);
    throw std::invalid_argument{"Pose file not found."};
  }

  beam_mapping::Poses poses_reader;
  if(!poses_reader.LoadFromFile(file_path)){
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
    throw std::runtime_error{"Invalid pose file."};
  }

  Eigen::Matrix4d T_MOVINGFRAME_BASELINK;
  if (!extrinsics_.GetT_SENSOR_BASELINK(T_MOVINGFRAME_BASELINK,
                                        poses_reader.GetMovingFrame())) {
    BEAM_ERROR("Cannot lookup extrinsics. Exiting.");
    throw std::runtime_error{"Cannot lookup extrinsics."};
  }

  std::vector<Eigen::Matrix4d, beam::AlignMat4d> transforms =
      poses_reader.GetPoses();
  std::vector<ros::Time> timestamps = poses_reader.GetTimeStamps();

  // create buffer core with cache time slightly larger than the difference
  // between the min and max timestamps. Also include a minimum
  int64_t cache_time =
      1.2 * (timestamps[timestamps.size() - 1].toSec() - timestamps[0].toSec());
  if (cache_time < 10) {
    cache_time = 10;
  }

  std::shared_ptr<tf2::BufferCore> poses = std::make_shared<tf2::BufferCore>(ros::Duration(cache_time));

  for (int i = 0; i < transforms.size(); i++) {
    const Eigen::Matrix4d& T_WORLD_MOVINGFRAME = transforms[i];
    Eigen::Matrix4d T_WORLD_BASELINK =
        T_WORLD_MOVINGFRAME * T_MOVINGFRAME_BASELINK;
    geometry_msgs::TransformStamped tf_stamped;
    bs_common::EigenTransformToTransformStampedMsg(
        T_WORLD_BASELINK, timestamps[i], i, extrinsics_.GetWorldFrameId(),
        extrinsics_.GetBaselinkFrameId(), tf_stamped);
    poses->setTransform(tf_stamped, authority_, false);
  }

  pose_lookup_ = std::make_shared<bs_common::PoseLookup>(poses);
}

}  // namespace frame_initializers
}  // namespace bs_models