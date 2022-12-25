#include <bs_models/frame_initializers/pose_file_frame_initializer.h>

#include <boost/filesystem.hpp>

#include <beam_mapping/Poses.h>
#include <beam_utils/log.h>

#include <bs_common/utils.h>

namespace bs_models { namespace frame_initializers {

PoseFileFrameInitializer::PoseFileFrameInitializer(
    const std::string& file_path) {
  authority_ = "poses_file";

  if (!boost::filesystem::exists(file_path)) {
    BEAM_ERROR("Pose file not found: {}", file_path);
    throw std::invalid_argument{"Pose file not found."};
  }

  beam_mapping::Poses poses_reader;
  if (!poses_reader.LoadFromFile(file_path)) {
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
  if (cache_time < 10) { cache_time = 10; }

  std::shared_ptr<tf2::BufferCore> poses =
      std::make_shared<tf2::BufferCore>(ros::Duration(cache_time));

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

bool PoseFileFrameInitializer::GetEstimatedPose(
    Eigen::Matrix4d& T_WORLD_SENSOR, const ros::Time& time,
    const std::string& sensor_frame_id, std::string& error_msg) {
  // get extrinsic
  Eigen::Matrix4d T_SENSOR_BASELINK;
  extrinsics_.GetT_SENSOR_BASELINK(T_SENSOR_BASELINK, sensor_frame_id);

  // get pose from pose lookup
  Eigen::Matrix4d T_WORLD_BASELINKcur;
  const auto success1 =
      pose_lookup_->GetT_WORLD_BASELINK(T_WORLD_BASELINKcur, time, error_msg);
  if (!success1) { return false; }

  // use normal pose lookup if path is empty or request is before the path
  const auto time_nsec = time.toNSec();
  if (path_poses_.empty() ||
      (!path_poses_.empty() && time_nsec < path_poses_.begin()->first)) {
    T_WORLD_SENSOR = T_WORLD_BASELINKcur * T_SENSOR_BASELINK.inverse();
    return true;
  }

  // get pose before or equal to the query in path
  const auto [time_path, T_WORLD_BASELINKpath] =
      *(std::prev(path_poses_.upper_bound(time.toNSec())));

  // if its an exact match, return the pose from the path
  if (time_path == time.toNSec()) {
    T_WORLD_SENSOR = T_WORLD_BASELINKpath * T_SENSOR_BASELINK.inverse();
    return true;
  }

  // get associated pose from the pose lookup
  ros::Time time_prev;
  time_prev.fromNSec(time_path);
  Eigen::Matrix4d T_WORLD_BASELINKprev;
  const auto success2 = pose_lookup_->GetT_WORLD_BASELINK(T_WORLD_BASELINKprev,
                                                          time_prev, error_msg);
  if (!success2) { return false; }

  // get relative transform using the pose lookup poses
  const auto T_BASELINKprev_BASELINKcur =
      T_WORLD_BASELINKprev.inverse() * T_WORLD_BASELINKcur;

  // apply relative pose to the pose from the path pose
  const auto T_WORLD_BASELINK =
      T_WORLD_BASELINKpath * T_BASELINKprev_BASELINKcur;

  // return result
  T_WORLD_SENSOR = T_WORLD_BASELINK * T_SENSOR_BASELINK.inverse();
  return true;
}

}} // namespace bs_models::frame_initializers