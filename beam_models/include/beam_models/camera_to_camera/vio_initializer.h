#pragma once

#include <beam_calibration/CameraModels.h>
#include <beam_calibration/ConvertCameraModel.h>
#include <beam_cv/geometry/Triangulation.h>
//#include <beam_cv/tracker/Tracker.h>

#include <sensor_msgs/Imu.h>

#include <beam_variables/position_3d.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

#include <beam_models/camera_to_camera/configurator.h>

#include <slamtools/common.h>
#include <slamtools/frame.h>
#include <slamtools/vig_initializer.h>
#include <slamtools/sliding_window.h>

namespace beam_models { namespace camera_to_camera {

class VIOInitializer {
public:
  VIOInitializer() = default;

  VIOInitializer(//std::shared_ptr<beam_cv::Tracker> tracker,
                 std::shared_ptr<beam_calibration::CameraModel> cam_model,
                 Eigen::Matrix4d T_body_cam, Eigen::Matrix4d T_body_imu,
                 Eigen::Vector4d imu_intrinsics);

  bool AddImage(cv::Mat cur_img, ros::Time cur_time);

  void AddIMU(Eigen::Vector3d ang_vel, Eigen::Vector3d lin_accel,
              ros::Time cur_time);

  void GetPose(fuse_variables::Orientation3DStamped::SharedPtr orientation,
                   fuse_variables::Position3DStamped::SharedPtr position);

  //std::shared_ptr<beam_cv::Tracker> GetTracker();

  std::unordered_map<uint64_t, fuse_variables::Position3D::SharedPtr>
      GetLandmarks();

  void GetBiases(Eigen::Vector3d& g, Eigen::Vector3d& bg, Eigen::Vector3d& ba);

private:
  std::unique_ptr<Frame> CreateFrame();

protected:
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  //std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<beam_calibration::UndistortImages> rectifier_;

  ros::Time current_frame_time_;

  std::unique_ptr<SlidingWindow> sliding_window_;
  std::unique_ptr<VigInitializer> initializer_;

  std::shared_ptr<beam_models::camera_to_camera::CameraConfig> config_;

  std::unique_ptr<Frame> pending_frame_;
  bool is_initialized = false;
};

}} // namespace beam_models::camera_to_camera
