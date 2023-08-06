#pragma once

#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/variable.h>
#include <fuse_variables/acceleration_linear_3d_stamped.h>
#include <fuse_variables/velocity_angular_3d_stamped.h>

#include <bs_common/imu_state.h>

#ifndef GRAVITY_NOMINAL
#  define GRAVITY_NOMINAL 9.80665
#endif


namespace bs_common {

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

/// @brief Gets a pointer to a gryo bias variable if it exists (nullptr
/// otherwise)
/// @param graph graph to look in
/// @param stamp of variable to get
/// @return pointer to variable
bs_variables::GyroscopeBias3DStamped::SharedPtr
    GetGryoscopeBias(fuse_core::Graph::ConstSharedPtr graph,
                     const ros::Time& stamp);

/// @brief Gets a pointer to an accel bias variable if it exists (nullptr
/// otherwise)
/// @param graph graph to look in
/// @param stamp of variable to get
/// @return pointer to variable
bs_variables::AccelerationBias3DStamped::SharedPtr
    GetAccelBias(fuse_core::Graph::ConstSharedPtr graph,
                 const ros::Time& stamp);

/// @brief Gets a pointer to a position variable if it exists (nullptr
/// otherwise)
/// @param graph graph to look in
/// @param stamp of variable to get
/// @return pointer to variable
fuse_variables::Position3DStamped::SharedPtr
    GetPosition(fuse_core::Graph::ConstSharedPtr graph, const ros::Time& stamp);

/// @brief Gets a pointer to an orientation variable if it exists (nullptr
/// otherwise)
/// @param graph graph to look in
/// @param stamp of variable to get
/// @return pointer to variable
fuse_variables::Orientation3DStamped::SharedPtr
    GetOrientation(fuse_core::Graph::ConstSharedPtr graph,
                   const ros::Time& stamp);

/// @brief Gets a pointer to a velocity variable if it exists (nullptr
/// otherwise)
/// @param graph graph to look in
/// @param stamp of variable to get
/// @return pointer to variable
fuse_variables::VelocityLinear3DStamped::SharedPtr
    GetVelocity(fuse_core::Graph::ConstSharedPtr graph, const ros::Time& stamp);

/// @brief Gets a pointer to an angular velocity variable if it exists (nullptr
/// otherwise)
/// @param graph graph to look in
/// @param stamp of variable to get
/// @return pointer to variable
fuse_variables::VelocityAngular3DStamped::SharedPtr
    GetAngularVelocity(fuse_core::Graph::ConstSharedPtr graph,
                       const ros::Time& stamp);

/// @brief Gets a pointer to a linear acceleration variable if it exists
/// (nullptr otherwise)
/// @param graph graph to look in
/// @param stamp of variable to get
/// @return pointer to variable
fuse_variables::AccelerationLinear3DStamped::SharedPtr
    GetLinearAcceleration(fuse_core::Graph::ConstSharedPtr graph,
                          const ros::Time& stamp);

/// @brief Gets all timestamps in the given graph
/// @param graph to search in
/// @return set of timestamps
std::set<ros::Time> CurrentTimestamps(fuse_core::Graph::ConstSharedPtr graph);

/// @brief Gets all landmark id's in the given graph
/// @param graph to search in
/// @return set of landmark ids
std::set<uint64_t> CurrentLandmarkIDs(fuse_core::Graph::ConstSharedPtr graph);

} // namespace bs_common
