#include <beam_common/utils.h>

namespace beam_common {

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

void FusePoseToEigenTransform(const fuse_variables::Position3DStamped& p,
                              const fuse_variables::Orientation3DStamped& o,
                              Eigen::Matrix4d& T_WORLD_SENSOR) {
  Eigen::Quaterniond q(o.w(), o.x(), o.y(), o.z());
  T_WORLD_SENSOR.block(0, 3, 3, 1) = Eigen::Vector3d{p.x(), p.y(), p.z()};
  T_WORLD_SENSOR.block(0, 0, 3, 3) = q.toRotationMatrix();
}

Eigen::Matrix4d FusePoseToEigenTransform(
    const fuse_variables::Position3DStamped& p,
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

void InterpolateTransformFromPath(const nav_msgs::Path& path,
                                  const ros::Time& time,
                                  Eigen::Matrix4d& T_WORLD_SENSOR) {
  for (int i = 0; i < path.poses.size(); i++) {
    if (time < path.poses[i + 1].header.stamp &&
        time >= path.poses[i].header.stamp) {
      Eigen::Matrix4d pose1, pose2;
      PoseMsgToTransformationMatrix(path.poses[i], pose1);
      PoseMsgToTransformationMatrix(path.poses[i + 1], pose2);
      T_WORLD_SENSOR = beam::InterpolateTransform(
          pose1, beam::RosTimeToChrono(path.poses[i].header.stamp), pose2,
          beam::RosTimeToChrono(path.poses[i + 1].header.stamp),
          beam::RosTimeToChrono(time));
    }
  }
}

bool MatchScans(
    const beam_common::ScanPose& scan_pose_1,
    const beam_common::ScanPose& scan_pose_2,
    const std::unique_ptr<
        beam_matching::Matcher<beam_matching::LoamPointCloudPtr>>& matcher,
    double outlier_threshold_r_deg, double outlier_threshold_t_m,
    Eigen::Matrix4d& T_CLOUD1_CLOUD2) {
  Eigen::Matrix4d T_CLOUD1_CLOUD2_init =
      beam::InvertTransform(scan_pose_1.T_REFFRAME_CLOUD()) *
      scan_pose_2.T_REFFRAME_CLOUD();

  beam_matching::LoamPointCloud cloud2_RefFInit = scan_pose_2.LoamCloud();
  cloud2_RefFInit.TransformPointCloud(T_CLOUD1_CLOUD2_init);
  std::shared_ptr<beam_matching::LoamPointCloud> c2 =
      std::make_shared<beam_matching::LoamPointCloud>(cloud2_RefFInit);
  std::shared_ptr<beam_matching::LoamPointCloud> c1 =
      std::make_shared<beam_matching::LoamPointCloud>(scan_pose_1.LoamCloud());

  matcher->SetRef(c2);
  matcher->SetTarget(c1);

  // match clouds
  if (!matcher->Match()) {
    ROS_ERROR("Failed scan matching. Skipping measurement.");
    return false;
  }

  Eigen::Matrix4d T_CLOUD1Est_CLOUD1Ini = matcher->GetResult().matrix();
  T_CLOUD1_CLOUD2 = T_CLOUD1Est_CLOUD1Ini * T_CLOUD1_CLOUD2_init;

  if (!beam::ArePosesEqual(T_CLOUD1_CLOUD2, T_CLOUD1_CLOUD2_init,
                           outlier_threshold_r_deg, outlier_threshold_t_m)) {
    ROS_ERROR(
        "Failed scan matcher transform threshold check. Skipping "
        "lidar keyframe.");
    return false;
  }

  return true;
}

double CalculateTrajectoryLength(
    const std::list<beam_common::ScanPose>& keyframes) {
  double length{0};
  auto iter = keyframes.begin();
  Eigen::Vector3d prev_position = iter->T_REFFRAME_CLOUD().block(0, 3, 3, 1);
  iter++;

  while (iter != keyframes.end()) {
    Eigen::Vector3d current_position =
        iter->T_REFFRAME_CLOUD().block(0, 3, 3, 1);
    Eigen::Vector3d current_motion = current_position - prev_position;
    length += current_motion.norm();
    prev_position = current_position;
    iter++;
  }

  return length;
}

}  // namespace beam_common