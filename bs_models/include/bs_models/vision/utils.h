#pragma once

#include <beam_calibration/CameraModel.h>
#include <beam_containers/LandmarkContainer.h>
#include <beam_cv/Utils.h>
#include <beam_utils/utils.h>

namespace bs_models { namespace vision {
    
/**
 * @brief Estimates a trajectory given visual measurements only
 * @param camera_model pointer to camera model being used
 * @param landmark_container container storing visual measurements
 * @param T_camera_baselink camera extrinsic
 * @param max_optimization_time maximum amount of time to optimize for
 * @return up to scale path estimate (T_WORLD_BASELINK)
 */
std::map<uint64_t, Eigen::Matrix4d> ComputePathWithVision(
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
    const std::shared_ptr<beam_containers::LandmarkContainer>&
        landmark_container,
    const Eigen::Matrix4d& T_camera_baselink,
    double max_optimization_time = 0.5, double keyframe_hz = 4.0);

}} // namespace bs_models::vision
