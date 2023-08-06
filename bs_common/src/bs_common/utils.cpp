#include <bs_common/utils.h>

#include <bs_common/conversions.h>

namespace bs_common {

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

} // namespace bs_common
