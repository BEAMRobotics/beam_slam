#pragma once

#include <beam_calibration/CameraModel.h>
#include <beam_containers/LandmarkContainer.h>
#include <beam_cv/Utils.h>
#include <beam_utils/utils.h>

namespace bs_models { namespace vision {

std::map<uint64_t, Eigen::Matrix4d> computePathWithVision(
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
    const std::shared_ptr<beam_containers::LandmarkContainer>& landmark_container,
    const Eigen::Matrix4d& T_camera_baselink, const std::deque<ros::Time>& img_times);

double
    computeParallax(const std::shared_ptr<beam_containers::LandmarkContainer>& landmark_container,
                    const ros::Time& t1, const ros::Time& t2);

}} // namespace bs_models::vision
