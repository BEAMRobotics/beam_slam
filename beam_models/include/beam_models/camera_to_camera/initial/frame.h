#pragma once

#include <beam_common/preintegrator.h>
#include <beam_utils/utils.h>

namespace beam_models { namespace camera_to_camera {

class Frame {
public:
  Frame(ros::Time stamp, Eigen::Vector3d p_WORLD_IMU,
        Eigen::Quaterniond q_WORLD_IMU,
        beam_common::PreIntegrator preintegrator);

  ros::Time t;
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  beam_common::PreIntegrator preint;
};

}} // namespace beam_models::camera_to_camera
