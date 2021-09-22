#pragma once

#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_core/transaction.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

#include <beam_matching/loam/LoamPointCloud.h>
#include <beam_utils/math.h>

#ifndef GRAVITY_NOMINAL
#define GRAVITY_NOMINAL 9.80665
#endif

static const Eigen::Vector3d GRAVITY_WORLD{0.0, 0.0, -GRAVITY_NOMINAL};

namespace bs_common {

void EigenTransformToFusePose(const Eigen::Matrix4d& T_WORLD_SENSOR,
                              fuse_variables::Position3DStamped& p,
                              fuse_variables::Orientation3DStamped& o);

void EigenTransformToFusePose(
    const Eigen::Matrix4d& T_WORLD_SENSOR,
    fuse_variables::Position3DStamped::SharedPtr& p,
    fuse_variables::Orientation3DStamped::SharedPtr& o);

void FusePoseToEigenTransform(const fuse_variables::Position3DStamped& p,
                              const fuse_variables::Orientation3DStamped& o,
                              Eigen::Matrix4d& T_WORLD_SENSOR);

Eigen::Matrix4d
    FusePoseToEigenTransform(const fuse_variables::Position3DStamped& p,
                             const fuse_variables::Orientation3DStamped& o);

void PoseMsgToTransformationMatrix(const geometry_msgs::PoseStamped& pose,
                                   Eigen::Matrix4d& T_WORLD_SENSOR);

void OdometryMsgToTransformationMatrix(const nav_msgs::Odometry& odom,
                                       Eigen::Matrix4d& T_WORLD_SENSOR);


void ROSStampedTransformToEigenTransform(const tf::StampedTransform& TROS,
                                         Eigen::Matrix4d& T);

void TransformStampedMsgToEigenTransform(
    const geometry_msgs::TransformStamped& TROS, Eigen::Matrix4d& T);

void EigenTransformToTransformStampedMsg(
    const Eigen::Matrix4d& T, const ros::Time& stamp, int seq,
    const std::string& parent_frame_id, const std::string& child_frame_id,
    geometry_msgs::TransformStamped& tf_stamped);

void OdometryMsgToTransformedStamped(
    const nav_msgs::Odometry& message, const ros::Time& stamp, int seq,
    const std::string& parent_frame_id, const std::string& child_frame_id,
    geometry_msgs::TransformStamped& tf_stamped);

/**
 * @brief Turns a pose message into an Eigen 4x4 matrix
 * @param pose pose message to turn into eigen matrix
 * @param T_WORLD_SENSOR[out] Transform to return
 */
void TransformationMatrixToPoseMsg(const Eigen::Matrix4d& T_WORLD_SENSOR,
                                   const ros::Time& stamp,
                                   geometry_msgs::PoseStamped& pose);

/**
 * @brief Interpolates a pose given a list of poses and a time
 * @param poses list of poses
 * @param time time to interpolate pose for
 * @param T_WORLD_SENSOR[out] pose to return
 */
void InterpolateTransformFromPath(
    const std::vector<geometry_msgs::PoseStamped>& poses, const ros::Time& time,
    Eigen::Matrix4d& T_WORLD_SENSOR);

/**
 * @brief Estimates a velocity at a time given a set of poses
 * @param poses list of poses
 * @param time time to interpolate pose for
 * @param velocity to return
 */
void EstimateVelocityFromPath(
    const std::vector<geometry_msgs::PoseStamped>& poses, const ros::Time& time,
    Eigen::Vector3d& velocity);

/**
 * @brief Get full path the the config root directory in beam_slam_launch
 * (.../beam_slam_launch/config/)
 * @return path
 */
std::string GetBeamSlamConfigPath();

/**
 * @brief Get number of constraints being added by a transaction
 * @param transaction
 * @return number of constraints
 */
int GetNumberOfConstraints(
    const fuse_core::Transaction::SharedPtr& transaction);

/**
 * @brief Get number of variables being added by a transaction
 * @param transaction
 * @return number of variables
 */
int GetNumberOfVariables(const fuse_core::Transaction::SharedPtr& transaction);

}  // namespace bs_common
