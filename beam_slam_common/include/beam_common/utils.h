#pragma once

#include <beam_utils/math.h>

#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>

#include <beam_matching/loam/LoamPointCloud.h>
#include <beam_common/scan_pose.h>

namespace beam_common {

static std::string default_string{""};

void EigenTransformToFusePose(const Eigen::Matrix4d& T_WORLD_SENSOR,
                              fuse_variables::Position3DStamped& p,
                              fuse_variables::Orientation3DStamped& o);

void FusePoseToEigenTransform(const fuse_variables::Position3DStamped& p,
                              const fuse_variables::Orientation3DStamped& o,
                              Eigen::Matrix4d& T_WORLD_SENSOR);

Eigen::Matrix4d FusePoseToEigenTransform(
    const fuse_variables::Position3DStamped& p,
    const fuse_variables::Orientation3DStamped& o);

void PoseMsgToTransformationMatrix(const geometry_msgs::PoseStamped& pose,
                                   Eigen::Matrix4d& T_WORLD_SENSOR);

void InterpolateTransformFromPath(const nav_msgs::Path& path,
                                  const ros::Time& time,
                                  Eigen::Matrix4d& T_WORLD_SENSOR);

/**
 * @brief iterates through all keypoints in the list and add up the change in
 * position between each keyframe.
 * @param keyframes list of scan poses that makeup the trajectory of keyframes
 * @return trajectory length
 */
double CalculateTrajectoryLength(
    const std::list<beam_common::ScanPose>& keyframes);

void ROSStampedTransformToEigenTransform(
    const tf::StampedTransform& TROS, Eigen::Matrix4d& T);

void GeometryTransformStampedToEigenTransform(
    const geometry_msgs::TransformStamped& TROS, Eigen::Matrix4d& T);

}  // namespace beam_common
