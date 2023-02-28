#pragma once

#include <bs_models/imu/imu_preintegration.h>

namespace bs_models { namespace imu {
/**
 * @brief Estimates inertial parameters given an initial path and imu messages
 * @param path initial path estimate of robot (T_world_baselink)
 * @param imu_buffer buffer of imu messages
 * @param params initial noise parameters of imu
 * @param gravity [out] output resulting gravity estimate
 * @param bg [out] output resulting  gyroscope bias
 * @param ba [out] output resulting accelerometer bias
 * @param velocities [out] map of velocities at each pose in the path
 * @param scale [out] scale estimate wrt the imu messages
 */
void EstimateParameters(const std::map<uint64_t, Eigen::Matrix4d>& path,
                        const std::deque<sensor_msgs::Imu>& imu_buffer,
                        const bs_models::ImuPreintegration::Params& params,
                        Eigen::Vector3d& gravity, Eigen::Vector3d& bg,
                        Eigen::Vector3d& ba,
                        std::map<uint64_t, Eigen::Vector3d>& velocities,
                        double& scale);

/**
 * @brief Estimates gyroscope bias given imu states
 * @param imu_frames list of imu states with populated imu buffers and external
 * poses
 * @param bg [out] resulting gyroscope bias
 */
void EstimateGyroBias(const std::vector<bs_common::ImuState>& imu_frames,
                      Eigen::Vector3d& bg);

/**
 * @brief Estimates gravity, scale and velocities
 * @param imu_frames list of imu states with populated imu buffers and external
 * poses
 * @param gravity [out] estimated gravity
 * @param scale [out] estimated scale
 * @param velocities [out] estimated velocities at each imu frame time
 */
void EstimateGravityScaleVelocities(
    const std::vector<bs_common::ImuState>& imu_frames,
    Eigen::Vector3d& gravity, double& scale,
    std::vector<std::pair<uint64_t, Eigen::Vector3d>>& velocities);

/**
 * @brief Refines gravity, scale and velocity
 * @param imu_frames list of imu states with populated imu buffers and external
 * poses
 * @param gravity [out] estimated gravity
 * @param scale [out] estimated scale
 * @param velocities [out] estimated velocities at each imu frame time
 */
void RefineGravityScaleVelocities(
    const std::vector<bs_common::ImuState>& imu_frames,
    Eigen::Vector3d& gravity, double& scale,
    std::vector<std::pair<uint64_t, Eigen::Vector3d>>& velocities);

/**
 * @brief Checks that the existing imu data is observable
 * @param imu_frames list of imu states with populated imu buffers and external
 * poses
 */
double ImuObservability(const std::vector<bs_common::ImuState>& imu_frames);

}} // namespace bs_models::imu