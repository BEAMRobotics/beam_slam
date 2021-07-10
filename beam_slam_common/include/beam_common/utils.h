#pragma once

#include <beam_utils/math.h>

#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <nav_msgs/Path.h>

#include <beam_matching/loam/LoamPointCloud.h>
#include <beam_common/scan_pose.h>

namespace beam_common {

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
 * @brief match two scan poses and get resulting transform between them
 * @param scan_pose_1
 * @param scan_pose_2
 * @param matcher
 * @param outlier_threshold_r_deg
 * @param outlier_threshold_t_m
 * @param T_CLOUD1_CLOUD2 reference to result (transform from cloud 2 to cloud
 * 1)
 * @return true if match was successful
 */
bool MatchScans(
    const beam_common::ScanPose& scan_pose_1,
    const beam_common::ScanPose& scan_pose_2,
    const std::unique_ptr<beam_matching::Matcher<beam_matching::LoamPointCloudPtr>>& matcher,
    double outlier_threshold_r_deg, double outlier_threshold_t_m,
    Eigen::Matrix4d& T_CLOUD1_CLOUD2);

/**
 * @brief iterates through all keypoints in the list and add up the change in
 * position between each keyframe.
 * @param keyframes list of scan poses that makeup the trajectory of keyframes
 * @return trajectory length
 */
double CalculateTrajectoryLength(
    const std::list<beam_common::ScanPose>& keyframes);

}  // namespace beam_common
