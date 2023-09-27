#include <bs_models/frame_initializers/frame_initializer.h>

#include <boost/algorithm/string.hpp>
#include <nlohmann/json.hpp>

#include <beam_mapping/Poses.h>
#include <beam_utils/filesystem.h>

#include <bs_common/conversions.h>

namespace bs_models {

FrameInitializer::FrameInitializer(const std::string& config_path) {
  // Load frame ids
  nlohmann::json J;
  if (!beam::ReadJson(config_path, J)) {
    throw std::runtime_error{"Invalid frame initializer config path"};
  }

  ROS_INFO_STREAM("Loading frame initializer from: " << config_path);

  std::string type;
  std::string info;
  std::string sensor_frame_id_override;
  std::vector<double> tf_override;
  int queue_size;
  int64_t poses_buffer_time;
  Eigen::Matrix4d T_ORIGINAL_OVERRIDE = Eigen::Matrix4d::Identity();

  try {
    type = J["type"];
  } catch (...) {
    ROS_ERROR("Missing or misspelt parameter: 'type'");
    throw std::runtime_error{"Missing or misspelt parameter: 'type'"};
  }

  try {
    info = J["info"];
  } catch (...) {
    ROS_ERROR("Missing or misspelt parameter: 'info'");
    throw std::runtime_error{"Missing or misspelt parameter: 'info'"};
  }

  try {
    sensor_frame_id_override = J["sensor_frame_id_override"];
  } catch (...) {
    ROS_ERROR("Missing or misspelt parameter: 'sensor_frame_id_override'");
    throw std::runtime_error{
        "Missing or misspelt parameter: 'sensor_frame_id_override'"};
  }

  try {
    for (const auto& value : J["T_ORIGINAL_OVERRIDE"]) {
      tf_override.push_back(value.get<double>());
    }
    if (tf_override.size() != 16) {
      ROS_WARN("Invalid T_ORIGINAL_OVERRIDE params, required 16 params, "
               "given: %zu. Using default identity transform",
               tf_override.size());
    } else {
      T_ORIGINAL_OVERRIDE = Eigen::Matrix4d(tf_override.data());
    }

  } catch (...) {
    ROS_ERROR("Missing or misspelt parameter: 'T_ORIGINAL_OVERRIDE'");
    throw std::runtime_error{
        "Missing or misspelt parameter: 'T_ORIGINAL_OVERRIDE'"};
  }

  try {
    queue_size = J["queue_size"];
  } catch (...) {
    ROS_ERROR("Missing or misspelt parameter: 'queue_size'");
    throw std::runtime_error{"Missing or misspelt parameter: 'queue_size'"};
  }

  try {
    poses_buffer_time = J["poses_buffer_time"];
  } catch (...) {
    ROS_ERROR("Missing or misspelt parameter: 'poses_buffer_time'");
    throw std::runtime_error{
        "Missing or misspelt parameter: 'poses_buffer_time'"};
  }

  // if type == posefile ->
  if (type == "POSEFILE") {
    InitializeFromPoseFile(info);
  } else if (type == "ODOMETRY") {
    poses_buffer_duration_ = ros::Duration(poses_buffer_time);
    authority_ = "odometry";
    poses_ = std::make_shared<tf2::BufferCore>(poses_buffer_duration_);
    pose_lookup_ = std::make_shared<bs_common::PoseLookup>(poses_);
    T_ORIGINAL_OVERRIDE_ = T_ORIGINAL_OVERRIDE;

    ros::NodeHandle n;
    odometry_subscriber_ = n.subscribe<nav_msgs::Odometry>(
        info, queue_size,
        boost::bind(&FrameInitializer::OdometryCallback, this, _1));
    path_subscriber_ = n.subscribe<nav_msgs::Path>(
        "/local_mapper/path_publisher/path", queue_size,
        boost::bind(&FrameInitializer::PathCallback, this, _1));

    if (!sensor_frame_id_override.empty()) {
      if (!extrinsics_.IsSensorFrameIdValid(sensor_frame_id_override)) {
        BEAM_ERROR("Sensor frame id override [{}] invalid. Exiting.",
                   sensor_frame_id_override);
        throw std::invalid_argument{"Invalid sensor frame id override."};
      } else {
        BEAM_INFO("Overriding sensor frame id in odometry messages to: {}",
                  sensor_frame_id_override);
        sensor_frame_id_ = sensor_frame_id_override;
        override_sensor_frame_id_ = true;
      }
    } else {
      sensor_frame_id_ = extrinsics_.GetBaselinkFrameId();
    }
  } else {
    ROS_ERROR("Invalid frame initializer type.");
    throw std::runtime_error{"Invalid frame initializer type."};
  }
}

bool FrameInitializer::GetPose(Eigen::Matrix4d& T_WORLD_SENSOR,
                               const ros::Time& time,
                               const std::string& sensor_frame_id,
                               std::string& error_msg) {
  if (graph_path_.empty()) {
    return pose_lookup_->GetT_WORLD_SENSOR(T_WORLD_SENSOR, sensor_frame_id,
                                           time, error_msg);
  }
  Eigen::Matrix4d T_BASELINK_SENSOR;
  extrinsics_.GetT_BASELINK_SENSOR(T_BASELINK_SENSOR, sensor_frame_id);

  // copy graph path
  path_mutex_.lock();
  Path graph_path_copy = graph_path_;
  path_mutex_.unlock();

  if (time < (*graph_path_copy.begin()).first) {
    error_msg = "Requested time is before the start of the current graph.";
    return false;
  }

  if (graph_path_copy.find(time) != graph_path_copy.end()) {
    // we assume the graph path is in the baselink frame
    Eigen::Matrix4d T_WORLD_BASELINK = graph_path_copy.at(time);
    T_WORLD_SENSOR = T_WORLD_BASELINK * T_BASELINK_SENSOR;
  } else {
    // get the graph pose that comes directly before current time (even if its
    // the end)
    auto lb = graph_path_copy.lower_bound(time);
    lb = std::prev(lb);
    const ros::Time closest_graph_time = lb->first;
    Eigen::Matrix4d T_WORLD_BASELINKprev =
        graph_path_copy.at(closest_graph_time);

    // compute relative pose between the graph pose and the current time
    Eigen::Matrix4d T_prev_now;
    if (!GetRelativePose(T_prev_now, closest_graph_time, time, error_msg)) {
      return false;
    }
    Eigen::Matrix4d T_WORLD_BASELINKnow = T_WORLD_BASELINKprev * T_prev_now;
    T_WORLD_SENSOR = T_WORLD_BASELINKnow * T_BASELINK_SENSOR;
  }
  return true;
}

bool FrameInitializer::GetRelativePose(Eigen::Matrix4d& T_A_B,
                                       const ros::Time& tA, const ros::Time& tB,
                                       std::string& error_msg) {
  // get pose at time a
  std::string error1;
  Eigen::Matrix4d p_WORLD_BASELINKa;
  const auto A_success =
      pose_lookup_->GetT_WORLD_BASELINK(p_WORLD_BASELINKa, tA, error1);
  // get pose at time b
  std::string error2;
  Eigen::Matrix4d T_WORLD_BASELINKB;
  const auto B_success =
      pose_lookup_->GetT_WORLD_BASELINK(T_WORLD_BASELINKB, tB, error2);

  if (!A_success || !B_success) {
    error_msg = "\n\tError 1: " + error1 + "\n\tError 2: " + error2;
    return false;
  }

  T_A_B = beam::InvertTransform(p_WORLD_BASELINKa) * T_WORLD_BASELINKB;
  return true;
}

void FrameInitializer::CheckOdometryFrameIDs(
    const nav_msgs::OdometryConstPtr message) {
  check_world_baselink_frames_ = false;

  std::string parent_frame_id = message->header.frame_id;
  std::string child_frame_id = message->child_frame_id;

  // check that parent frame supplied by odometry contains world frame
  if (!boost::algorithm::contains(parent_frame_id,
                                  extrinsics_.GetWorldFrameId())) {
    BEAM_WARN(
        "World frame in extrinsics does not match parent frame in odometry "
        "messages. Using extrinsics.");
  }

  // check that child frame supplied by odometry contains one of the sensor
  // frames
  if (!override_sensor_frame_id_) {
    if (boost::algorithm::contains(child_frame_id,
                                   extrinsics_.GetImuFrameId())) {
      sensor_frame_id_ = extrinsics_.GetImuFrameId();
    } else if (boost::algorithm::contains(child_frame_id,
                                          extrinsics_.GetCameraFrameId())) {
      sensor_frame_id_ = extrinsics_.GetCameraFrameId();
    } else if (boost::algorithm::contains(child_frame_id,
                                          extrinsics_.GetLidarFrameId())) {
      sensor_frame_id_ = extrinsics_.GetLidarFrameId();
    } else {
      BEAM_ERROR("Sensor frame id in odometry message ({}) not equal to any "
                 "sensor frame in extrinsics. Please provide a "
                 "sensor_frame_id_override. Available sensor frame ids: {}",
                 child_frame_id, extrinsics_.GetFrameIdsString());
      throw std::invalid_argument{"Invalid frame id"};
    }
  }
}

void FrameInitializer::OdometryCallback(
    const nav_msgs::OdometryConstPtr message) {
  if (check_world_baselink_frames_) { CheckOdometryFrameIDs(message); }

  // if sensor_frame is already baselink, then we can directly copy
  if (sensor_frame_id_ == extrinsics_.GetBaselinkFrameId()) {
    geometry_msgs::TransformStamped tf_stamped;
    bs_common::OdometryMsgToTransformedStamped(
        *message, message->header.stamp, message->header.seq,
        extrinsics_.GetWorldFrameId(), extrinsics_.GetBaselinkFrameId(),
        tf_stamped);
    poses_->setTransform(tf_stamped, authority_, false);
    return;
  }

  // Otherwise, transform sensor frame to baselink and then publish
  Eigen::Matrix4d T_SENSOR_BASELINK;
  if (extrinsics_.GetT_SENSOR_BASELINK(T_SENSOR_BASELINK, sensor_frame_id_,
                                       message->header.stamp)) {
    Eigen::Matrix4d T_WORLD_ORIGINAL;
    bs_common::OdometryMsgToTransformationMatrix(*message, T_WORLD_ORIGINAL);

    Eigen::Matrix4d T_WORLD_SENSOR = T_WORLD_ORIGINAL * T_ORIGINAL_OVERRIDE_;

    Eigen::Matrix4d T_WORLD_BASELINK = T_WORLD_SENSOR * T_SENSOR_BASELINK;

    geometry_msgs::TransformStamped tf_stamped;
    bs_common::EigenTransformToTransformStampedMsg(
        T_WORLD_BASELINK, message->header.stamp, message->header.seq,
        extrinsics_.GetWorldFrameId(), extrinsics_.GetBaselinkFrameId(),
        tf_stamped);
    poses_->setTransform(tf_stamped, authority_, false);
    return;
  } else {
    BEAM_WARN("Skipping odometry message.");
    return;
  }
}

void FrameInitializer::PathCallback(const nav_msgs::PathConstPtr message) {
  path_mutex_.lock();
  graph_path_.clear();
  for (const auto& pose : message->poses) {
    const ros::Time stamp = pose.header.stamp;
    Eigen::Matrix4d T;
    bs_common::PoseMsgToTransformationMatrix(pose, T);
    graph_path_.insert({stamp, T});
  }
  path_mutex_.unlock();
}

void FrameInitializer::InitializeFromPoseFile(const std::string& file_path) {
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

  const std::vector<Eigen::Matrix4d, beam::AlignMat4d>& transforms =
      poses_reader.GetPoses();
  const std::vector<ros::Time>& timestamps = poses_reader.GetTimeStamps();

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

} // namespace bs_models