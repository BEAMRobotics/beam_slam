#include <beam_models/camera_to_camera/initial/frame.h>

namespace beam_models { namespace camera_to_camera {

Frame::Frame(ros::Time stamp, Eigen::Vector3d p_WORLD_IMU,
             Eigen::Quaterniond q_WORLD_IMU,
             beam_common::PreIntegrator preintegrator)
    : t(stamp), p(p_WORLD_IMU), q(q_WORLD_IMU), preint(preintegrator) {}

}} // namespace beam_models::camera_to_camera
