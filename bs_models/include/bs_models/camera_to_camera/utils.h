#pragma once

#include <ceres/ceres.h>

// libbeam
#include <beam_calibration/CameraModel.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/PoseRefinement.h>
#include <beam_cv/matchers/Matchers.h>
#include <beam_cv/trackers/Trackers.h>
#include <beam_utils/utils.h>
#include <bs_common/submap.h>
#include <bs_models/camera_to_camera/visual_map.h>

namespace bs_models { namespace camera_to_camera {

std::shared_ptr<beam_cv::PoseRefinement> PoseRefiner();

beam::opt<Eigen::Matrix4d> LocalizeFrame(
    const std::shared_ptr<beam_cv::Tracker>& tracker,
    const std::shared_ptr<bs_models::camera_to_camera::VisualMap>& visual_map,
    const std::shared_ptr<beam_cv::PoseRefinement>& pose_refiner,
    const std::shared_ptr<beam_calibration::CameraModel>& cam_model,
    const ros::Time& frame_time);

std::map<uint64_t, Eigen::Vector3d> MatchFrameToCurrentSubmap(
    const std::shared_ptr<beam_cv::Tracker>& tracker,
    const std::shared_ptr<bs_models::camera_to_camera::VisualMap>& visual_map,
    const std::shared_ptr<beam_calibration::CameraModel>& cam_model,
    const ros::Time& frame_time);

double ComputeAvgParallax(const std::shared_ptr<beam_cv::Tracker>& tracker,
                          const ros::Time& t1, const ros::Time& t2);

}} // namespace bs_models::camera_to_camera
