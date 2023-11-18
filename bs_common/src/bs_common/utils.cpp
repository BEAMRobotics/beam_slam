#include <bs_common/utils.h>

#include <fuse_constraints/relative_constraint.h>
#include <fuse_constraints/relative_pose_3d_stamped_constraint.h>

#include <beam_utils/filesystem.h>

#include <bs_common/conversions.h>

namespace bs_common {

std::string ToString(const ros::Time& time) {
  std::string nsec = std::to_string(time.nsec);
  while (nsec.size() < 9) { nsec = "0" + nsec; }
  return std::to_string(time.sec) + "." + nsec;
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

double ShannonEntropyFromPoseCovariance(
    const Eigen::Matrix<double, 6, 6>& covariance) {
  double k = pow(2.0 * M_PI * std::exp(1.0), covariance.rows());
  double h = 0.5 * std::log(k * covariance.determinant());
  return h;
}

void AddZeroMotionFactor(const std::string& source,
                         const bs_common::ImuState& state1,
                         const bs_common::ImuState& state2,
                         fuse_core::Transaction::SharedPtr transaction) {
  // add all variables in case they dont already exist
  transaction->addVariable(
      std::make_shared<fuse_variables::Orientation3DStamped>(
          state1.Orientation()));
  transaction->addVariable(
      std::make_shared<fuse_variables::Position3DStamped>(state1.Position()));
  transaction->addVariable(
      std::make_shared<fuse_variables::VelocityLinear3DStamped>(
          state1.Velocity()));
  transaction->addVariable(
      std::make_shared<bs_variables::AccelerationBias3DStamped>(
          state1.AccelBias()));
  transaction->addVariable(
      std::make_shared<bs_variables::GyroscopeBias3DStamped>(
          state1.GyroBias()));

  transaction->addVariable(
      std::make_shared<fuse_variables::Orientation3DStamped>(
          state2.Orientation()));
  transaction->addVariable(
      std::make_shared<fuse_variables::Position3DStamped>(state2.Position()));
  transaction->addVariable(
      std::make_shared<fuse_variables::VelocityLinear3DStamped>(
          state2.Velocity()));
  transaction->addVariable(
      std::make_shared<bs_variables::AccelerationBias3DStamped>(
          state2.AccelBias()));
  transaction->addVariable(
      std::make_shared<bs_variables::GyroscopeBias3DStamped>(
          state2.GyroBias()));

  // generate a zero motion constraint
  fuse_core::Vector7d pose_delta;
  pose_delta << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  fuse_core::Matrix6d pose_covariance = fuse_core::Matrix6d::Identity() * 1e-6;

  auto relative_pose_constraint =
      std::make_shared<fuse_constraints::RelativePose3DStampedConstraint>(
          source, state1.Position(), state1.Orientation(), state2.Position(),
          state2.Orientation(), pose_delta, pose_covariance);
  transaction->addConstraint(relative_pose_constraint);

  fuse_core::Vector3d relative_delta;
  relative_delta << 0.0, 0.0, 0.0;
  fuse_core::Matrix3d relative_covariance =
      fuse_core::Matrix3d::Identity() * 1e-2;

  auto relative_vel_constraint =
      std::make_shared<fuse_constraints::RelativeConstraint<
          fuse_variables::VelocityLinear3DStamped>>(
          source, state1.Velocity(), state2.Velocity(), relative_delta,
          relative_covariance);
  transaction->addConstraint(relative_vel_constraint);

  auto relative_bg_constraint =
      std::make_shared<fuse_constraints::RelativeConstraint<
          bs_variables::GyroscopeBias3DStamped>>(
          source, state1.GyroBias(), state2.GyroBias(), relative_delta,
          relative_covariance);
  transaction->addConstraint(relative_bg_constraint);

  auto relative_ba_constraint =
      std::make_shared<fuse_constraints::RelativeConstraint<
          bs_variables::AccelerationBias3DStamped>>(
          source, state1.AccelBias(), state2.AccelBias(), relative_delta,
          relative_covariance);
  transaction->addConstraint(relative_ba_constraint);
}

std::string GetAbsoluteConfigPathFromJson(const std::string& json_path,
                                          const std::string& field_name) {
  nlohmann::json J;
  if (!beam::ReadJson(json_path, J)) {
    BEAM_ERROR("Unable to json");
    throw std::runtime_error{"Unable to read config"};
  }

  beam::ValidateJsonKeysOrThrow(std::vector<std::string>{field_name}, J);

  std::string path_rel = J[field_name];
  if (path_rel.empty()) {
    return "";
  } else {
    return beam::CombinePaths(GetBeamSlamConfigPath(), path_rel);
  }
}

} // namespace bs_common
