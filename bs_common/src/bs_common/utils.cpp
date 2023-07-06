#include <bs_common/utils.h>

namespace bs_common {

void EigenTransformToFusePose(const Eigen::Matrix4d& T_WORLD_SENSOR,
                              fuse_variables::Position3DStamped& p,
                              fuse_variables::Orientation3DStamped& o) {
  // get position
  p.x() = T_WORLD_SENSOR(0, 3);
  p.y() = T_WORLD_SENSOR(1, 3);
  p.z() = T_WORLD_SENSOR(2, 3);

  // get rotation
  Eigen::Matrix3d R = T_WORLD_SENSOR.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  o.x() = q.x();
  o.y() = q.y();
  o.z() = q.z();
  o.w() = q.w();
}

void EigenTransformToFusePose(
    const Eigen::Matrix4d& T_WORLD_SENSOR,
    fuse_variables::Position3DStamped::SharedPtr& p,
    fuse_variables::Orientation3DStamped::SharedPtr& o) {
  fuse_variables::Position3DStamped p_tmp;
  fuse_variables::Orientation3DStamped o_tmp;
  EigenTransformToFusePose(T_WORLD_SENSOR, p_tmp, o_tmp);

  p = fuse_variables::Position3DStamped::make_shared(p_tmp);
  o = fuse_variables::Orientation3DStamped::make_shared(o_tmp);
}

void FusePoseToEigenTransform(const fuse_variables::Position3DStamped& p,
                              const fuse_variables::Orientation3DStamped& o,
                              Eigen::Matrix4d& T_WORLD_SENSOR) {
  Eigen::Quaterniond q(o.w(), o.x(), o.y(), o.z());
  T_WORLD_SENSOR.block(0, 3, 3, 1) = Eigen::Vector3d{p.x(), p.y(), p.z()};
  T_WORLD_SENSOR.block(0, 0, 3, 3) = q.toRotationMatrix();
}

Eigen::Matrix4d
    FusePoseToEigenTransform(const fuse_variables::Position3DStamped& p,
                             const fuse_variables::Orientation3DStamped& o) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  // add position
  T(0, 3) = p.x();
  T(1, 3) = p.y();
  T(2, 3) = p.z();

  // add rotation
  Eigen::Quaterniond q(o.w(), o.x(), o.y(), o.z());
  Eigen::Matrix3d R = q.toRotationMatrix();
  T.block(0, 0, 3, 3) = R;
  return T;
}

void PoseMsgToTransformationMatrix(const geometry_msgs::PoseStamped& pose,
                                   Eigen::Matrix4d& T_WORLD_SENSOR) {
  Eigen::Vector3d position;
  position[0] = pose.pose.position.x;
  position[1] = pose.pose.position.y;
  position[2] = pose.pose.position.z;
  Eigen::Quaterniond orientation;
  orientation.w() = pose.pose.orientation.w;
  orientation.x() = pose.pose.orientation.x;
  orientation.y() = pose.pose.orientation.y;
  orientation.z() = pose.pose.orientation.z;
  beam::QuaternionAndTranslationToTransformMatrix(orientation, position,
                                                  T_WORLD_SENSOR);
}

void OdometryMsgToTransformationMatrix(const nav_msgs::Odometry& odom,
                                       Eigen::Matrix4d& T_WORLD_SENSOR) {
  T_WORLD_SENSOR = Eigen::Matrix4d::Identity();
  T_WORLD_SENSOR(0, 3) = odom.pose.pose.position.x;
  T_WORLD_SENSOR(1, 3) = odom.pose.pose.position.y;
  T_WORLD_SENSOR(2, 3) = odom.pose.pose.position.z;
  Eigen::Quaterniond q;
  q.x() = odom.pose.pose.orientation.x;
  q.y() = odom.pose.pose.orientation.y;
  q.z() = odom.pose.pose.orientation.z;
  q.w() = odom.pose.pose.orientation.w;
  Eigen::Matrix3d R = q.toRotationMatrix();
  T_WORLD_SENSOR.block(0, 0, 3, 3) = R;
}

nav_msgs::Odometry TransformToOdometryMessage(
    const ros::Time& stamp, const int seq, const std::string& parent_frame_id,
    const std::string& child_frame_id, const Eigen::Matrix4d T_PARENT_CHILD,
    const Eigen::Matrix<double, 6, 6> covariance) {
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.seq = seq;
  odom_msg.header.frame_id = parent_frame_id;
  odom_msg.child_frame_id = child_frame_id;
  // handle position
  geometry_msgs::Point position;
  position.x = T_PARENT_CHILD(0, 3);
  position.y = T_PARENT_CHILD(1, 3);
  position.z = T_PARENT_CHILD(2, 3);
  odom_msg.pose.pose.position = position;
  // handle orientation
  Eigen::Matrix3d R = T_PARENT_CHILD.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  geometry_msgs::Quaternion orientation;
  orientation.x = q.x();
  orientation.y = q.y();
  orientation.z = q.z();
  orientation.w = q.w();
  odom_msg.pose.pose.orientation = orientation;
  // handle covariance
  std::vector<double> cov(covariance.data(),
                          covariance.data() +
                              covariance.rows() * covariance.cols());
  for (int i = 0; i < cov.size(); i++) { odom_msg.pose.covariance[i] = cov[i]; }

  return odom_msg;
}

void EstimateVelocityFromPath(
    const std::vector<geometry_msgs::PoseStamped>& poses, const ros::Time& time,
    Eigen::Vector3d& velocity) {
  // estimate poses using the init path at
  Eigen::Matrix4d T_WORLD_BASELINK;
  InterpolateTransformFromPath(poses, time, T_WORLD_BASELINK);
  ros::Time stamp_plus_delta(time.toSec() + 0.1);
  Eigen::Matrix4d T_WORLD_BASELINK_plus;
  InterpolateTransformFromPath(poses, stamp_plus_delta, T_WORLD_BASELINK_plus);
  // get positions of each pose
  Eigen::Vector3d position_plus =
      T_WORLD_BASELINK_plus.block<3, 1>(0, 3).transpose();
  Eigen::Vector3d position = T_WORLD_BASELINK.block<3, 1>(0, 3).transpose();
  // compute velocity
  velocity =
      (position_plus - position) / (stamp_plus_delta.toSec() - time.toSec());
}

void ROSStampedTransformToEigenTransform(const tf::StampedTransform& TROS,
                                         Eigen::Matrix4d& T) {
  Eigen::Matrix4f T_float{Eigen::Matrix4f::Identity()};
  T_float(0, 3) = TROS.getOrigin().getX();
  T_float(1, 3) = TROS.getOrigin().getY();
  T_float(2, 3) = TROS.getOrigin().getZ();
  Eigen::Quaternionf q;
  q.x() = TROS.getRotation().getX();
  q.y() = TROS.getRotation().getY();
  q.z() = TROS.getRotation().getZ();
  q.w() = TROS.getRotation().getW();
  T_float.block(0, 0, 3, 3) = q.toRotationMatrix();
  T = T_float.cast<double>();
}

void TransformStampedMsgToEigenTransform(
    const geometry_msgs::TransformStamped& TROS, Eigen::Matrix4d& T) {
  Eigen::Matrix4f T_float{Eigen::Matrix4f::Identity()};
  T_float(0, 3) = TROS.transform.translation.x;
  T_float(1, 3) = TROS.transform.translation.y;
  T_float(2, 3) = TROS.transform.translation.z;
  Eigen::Quaternionf q;
  q.x() = TROS.transform.rotation.x;
  q.y() = TROS.transform.rotation.y;
  q.z() = TROS.transform.rotation.z;
  q.w() = TROS.transform.rotation.w;
  T_float.block(0, 0, 3, 3) = q.toRotationMatrix();
  T = T_float.cast<double>();
}

void EigenTransformToTransformStampedMsg(
    const Eigen::Matrix4d& T, const ros::Time& stamp, int seq,
    const std::string& parent_frame_id, const std::string& child_frame_id,
    geometry_msgs::TransformStamped& tf_stamped) {
  tf_stamped.header.stamp = stamp;
  tf_stamped.header.seq = seq;
  tf_stamped.header.frame_id = parent_frame_id;
  tf_stamped.child_frame_id = child_frame_id;
  tf_stamped.transform.translation.x = T(0, 3);
  tf_stamped.transform.translation.y = T(1, 3);
  tf_stamped.transform.translation.z = T(2, 3);
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  tf_stamped.transform.rotation.x = q.x();
  tf_stamped.transform.rotation.y = q.y();
  tf_stamped.transform.rotation.z = q.z();
  tf_stamped.transform.rotation.w = q.w();
}

void EigenTransformToOdometryMsg(const Eigen::Matrix4d& T,
                                 const ros::Time& stamp, int seq,
                                 const std::string& parent_frame_id,
                                 const std::string& child_frame_id,
                                 nav_msgs::Odometry& odom_msg) {
  odom_msg.header.seq = seq;
  odom_msg.header.frame_id = parent_frame_id;
  odom_msg.header.stamp = stamp;
  odom_msg.child_frame_id = child_frame_id;
  odom_msg.pose.pose.position.x = T(0, 3);
  odom_msg.pose.pose.position.y = T(1, 3);
  odom_msg.pose.pose.position.z = T(2, 3);
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();
}

void EigenTransformToPoseStamped(const Eigen::Matrix4d& T,
                                 const ros::Time& stamp, int seq,
                                 const std::string& frame_id,
                                 geometry_msgs::PoseStamped& pose_stamped) {
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.seq = seq;
  header.stamp = stamp;

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

  pose_stamped.header = header;
  pose_stamped.pose.position = position;
  pose_stamped.pose.orientation = orientation;
}

void OdometryMsgToTransformedStamped(
    const nav_msgs::Odometry& message, const ros::Time& stamp, int seq,
    const std::string& parent_frame_id, const std::string& child_frame_id,
    geometry_msgs::TransformStamped& tf_stamped) {
  tf_stamped.header.stamp = stamp;
  tf_stamped.header.seq = seq;
  tf_stamped.header.frame_id = parent_frame_id;
  tf_stamped.child_frame_id = child_frame_id;
  tf_stamped.transform.translation.x = message.pose.pose.position.x;
  tf_stamped.transform.translation.y = message.pose.pose.position.y;
  tf_stamped.transform.translation.z = message.pose.pose.position.z;
  tf_stamped.transform.rotation = message.pose.pose.orientation;
}

void TransformationMatrixToPoseMsg(const Eigen::Matrix4d& T_WORLD_SENSOR,
                                   const ros::Time& stamp,
                                   geometry_msgs::PoseStamped& pose) {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  beam::TransformMatrixToQuaternionAndTranslation(T_WORLD_SENSOR, orientation,
                                                  position);
  pose.header.stamp = stamp;
  pose.pose.position.x = position[0];
  pose.pose.position.y = position[1];
  pose.pose.position.z = position[2];
  pose.pose.orientation.w = orientation.w();
  pose.pose.orientation.x = orientation.x();
  pose.pose.orientation.y = orientation.y();
  pose.pose.orientation.z = orientation.z();
}

void InterpolateTransformFromPath(
    const std::vector<geometry_msgs::PoseStamped>& poses, const ros::Time& time,
    Eigen::Matrix4d& T_WORLD_SENSOR) {
  for (int i = 0; i < poses.size(); i++) {
    if (time < poses[i + 1].header.stamp && time >= poses[i].header.stamp) {
      Eigen::Matrix4d pose1, pose2;
      PoseMsgToTransformationMatrix(poses[i], pose1);
      PoseMsgToTransformationMatrix(poses[i + 1], pose2);
      T_WORLD_SENSOR = beam::InterpolateTransform(
          pose1, beam::RosTimeToChrono(poses[i].header.stamp), pose2,
          beam::RosTimeToChrono(poses[i + 1].header.stamp),
          beam::RosTimeToChrono(time));
    }
  }
}

std::string GetBeamSlamConfigPath() {
  std::string current_path_from_beam_slam = "bs_common/src/bs_common/utils.cpp";
  std::string config_root_location = __FILE__;
  config_root_location.erase(config_root_location.end() -
                                 current_path_from_beam_slam.length(),
                             config_root_location.end());
  config_root_location += "beam_slam_launch/config/";
  if (!boost::filesystem::exists(config_root_location)) {
    BEAM_ERROR("Cannot locate beam slam config folder. Expected to be at: {}",
               config_root_location);
  }
  return config_root_location;
}

std::string GetBeamSlamCalibrationsPath() {
  std::string current_path_from_beam_slam = "bs_common/src/bs_common/utils.cpp";
  std::string calibration_root_location = __FILE__;
  calibration_root_location.erase(calibration_root_location.end() -
                                      current_path_from_beam_slam.length(),
                                  calibration_root_location.end());
  calibration_root_location += "beam_slam_launch/calibrations/";
  if (!boost::filesystem::exists(calibration_root_location)) {
    BEAM_ERROR(
        "Cannot locate beam slam calibrations folder. Expected to be at: {}",
        calibration_root_location);
  }
  return calibration_root_location;
}

int GetNumberOfConstraints(
    const fuse_core::Transaction::SharedPtr& transaction) {
  if (transaction == nullptr) { return 0; }

  int counter = 0;
  auto added_constraints = transaction->addedConstraints();
  for (auto iter = added_constraints.begin(); iter != added_constraints.end();
       iter++) {
    counter++;
  }
  return counter;
}

int GetNumberOfVariables(const fuse_core::Transaction::SharedPtr& transaction) {
  if (transaction == nullptr) { return 0; }

  int counter = 0;
  auto added_variables = transaction->addedVariables();
  for (auto iter = added_variables.begin(); iter != added_variables.end();
       iter++) {
    counter++;
  }
  return counter;
}

bs_variables::GyroscopeBias3DStamped::SharedPtr
    GetGryoscopeBias(fuse_core::Graph::ConstSharedPtr graph,
                     const ros::Time& stamp) {
  auto gyro_bias = bs_variables::GyroscopeBias3DStamped::make_shared();
  const auto bg_uuid =
      fuse_core::uuid::generate(gyro_bias->type(), stamp, fuse_core::uuid::NIL);
  try {
    *gyro_bias = dynamic_cast<const bs_variables::GyroscopeBias3DStamped&>(
        graph->getVariable(bg_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return gyro_bias;
}

bs_variables::AccelerationBias3DStamped::SharedPtr
    GetAccelBias(fuse_core::Graph::ConstSharedPtr graph,
                 const ros::Time& stamp) {
  auto accel_bias = bs_variables::AccelerationBias3DStamped::make_shared();
  const auto ba_uuid = fuse_core::uuid::generate(accel_bias->type(), stamp,
                                                 fuse_core::uuid::NIL);
  try {
    *accel_bias = dynamic_cast<const bs_variables::AccelerationBias3DStamped&>(
        graph->getVariable(ba_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return accel_bias;
}

fuse_variables::Position3DStamped::SharedPtr
    GetPosition(fuse_core::Graph::ConstSharedPtr graph,
                const ros::Time& stamp) {
  auto position = fuse_variables::Position3DStamped::make_shared();
  const auto pos_uuid =
      fuse_core::uuid::generate(position->type(), stamp, fuse_core::uuid::NIL);
  try {
    *position = dynamic_cast<const fuse_variables::Position3DStamped&>(
        graph->getVariable(pos_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return position;
}

fuse_variables::Orientation3DStamped::SharedPtr
    GetOrientation(fuse_core::Graph::ConstSharedPtr graph,
                   const ros::Time& stamp) {
  auto orientation = fuse_variables::Orientation3DStamped::make_shared();
  const auto or_uuid = fuse_core::uuid::generate(orientation->type(), stamp,
                                                 fuse_core::uuid::NIL);
  try {
    *orientation = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
        graph->getVariable(or_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return orientation;
}

fuse_variables::VelocityLinear3DStamped::SharedPtr
    GetVelocity(fuse_core::Graph::ConstSharedPtr graph,
                const ros::Time& stamp) {
  auto velocity = fuse_variables::VelocityLinear3DStamped::make_shared();
  const auto vel_uuid =
      fuse_core::uuid::generate(velocity->type(), stamp, fuse_core::uuid::NIL);
  try {
    *velocity = dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(
        graph->getVariable(vel_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return velocity;
}

fuse_variables::VelocityAngular3DStamped::SharedPtr
    GetAngularVelocity(fuse_core::Graph::ConstSharedPtr graph,
                       const ros::Time& stamp) {
  auto velocity = fuse_variables::VelocityAngular3DStamped::make_shared();
  const auto vel_uuid =
      fuse_core::uuid::generate(velocity->type(), stamp, fuse_core::uuid::NIL);
  try {
    *velocity = dynamic_cast<const fuse_variables::VelocityAngular3DStamped&>(
        graph->getVariable(vel_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return velocity;
}

fuse_variables::AccelerationLinear3DStamped::SharedPtr
    GetLinearAcceleration(fuse_core::Graph::ConstSharedPtr graph,
                          const ros::Time& stamp) {
  auto velocity = fuse_variables::AccelerationLinear3DStamped::make_shared();
  const auto vel_uuid =
      fuse_core::uuid::generate(velocity->type(), stamp, fuse_core::uuid::NIL);
  try {
    *velocity =
        dynamic_cast<const fuse_variables::AccelerationLinear3DStamped&>(
            graph->getVariable(vel_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return velocity;
}

std::set<ros::Time> CurrentTimestamps(fuse_core::Graph::ConstSharedPtr graph) {
  std::set<ros::Time> times;
  for (auto& var : graph->getVariables()) {
    auto position = fuse_variables::Position3DStamped::make_shared();
    if (var.type() != position->type()) { continue; }
    *position = dynamic_cast<const fuse_variables::Position3DStamped&>(var);
    times.insert(position->stamp());
  }
  return times;
}

std::set<uint64_t> CurrentLandmarkIDs(fuse_core::Graph::ConstSharedPtr graph) {
  std::set<uint64_t> ids;
  for (auto& var : graph->getVariables()) {
    auto landmark = fuse_variables::Point3DLandmark::make_shared();
    if (var.type() != landmark->type()) { continue; }
    *landmark = dynamic_cast<const fuse_variables::Point3DLandmark&>(var);
    ids.insert(landmark->id());
  }
  return ids;
}

} // namespace bs_common
