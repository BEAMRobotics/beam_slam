#pragma once
// libbeam
#include <beam_calibration/CameraModels.h>
#include <beam_calibration/ConvertCameraModel.h>
#include <beam_cv/Utils.h>
#include <beam_cv/geometry/Triangulation.h>
// ros
#include <sensor_msgs/Imu.h>
// beam_slam
#include <beam_models/camera_to_camera/configurator.h>
#include <beam_models/camera_to_camera/opencvimage.h>
#include <beam_variables/position_3d.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
// slamtools
#include <slamtools/common.h>
#include <slamtools/frame.h>
#include <slamtools/sliding_window.h>
#include <slamtools/state.h>
#include <slamtools/vig_initializer.h>

namespace beam_models { namespace camera_to_camera {

typedef std::map<
    uint64_t, Eigen::Matrix4d, std::less<uint64_t>,
    Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Matrix4d>>>
    PoseMap;

class VIOInitializer {
public:
  VIOInitializer() = default;

  /**
   * @brief Custom Constructor
   */
  VIOInitializer(std::shared_ptr<beam_calibration::CameraModel> input_cam_model,
                 const Eigen::Matrix4d& T_imu_cam,
                 const Eigen::Vector4d& imu_intrinsics);

  /**
   * @brief Adds an image to the initializer, returns pass or fail
   * @param cur_img image to add
   * @param cur_time timestamp of image
   */
  bool AddImage(cv::Mat cur_img, ros::Time cur_time);

  /**
   * @brief Adds an imu measurement to the initializer
   * @param ang_vel angular velocity
   * @param lin_accel linear acceleration
   * @param cur_time timestamp of imu measurement
   */
  void AddIMU(const Eigen::Vector3d& ang_vel, const Eigen::Vector3d& lin_accel,
              ros::Time cur_time);

  /**
   * @brief Returns the map of poses in sliding window, in imu frame
   */
  PoseMap GetPoses();

  /**
   * @brief Fills the estimated bias parameters
   * @param v velocity
   * @param bg gyro bias
   * @param bas accelerometer bias
   */
  void GetBiases(Eigen::Vector3d& v, Eigen::Vector3d& bg, Eigen::Vector3d& ba);

  /**
   * @brief Returns the current state
   */
  bool Initialized();

private:
  /**
   * @brief Creates a new frame object
   */
  std::unique_ptr<Frame> CreateFrame();

protected:
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_calibration::UndistortImages> rectifier_;
  std::shared_ptr<CameraConfig> config_;
  std::unique_ptr<SlidingWindow> sliding_window_;
  std::unique_ptr<VigInitializer> initializer_;
  std::unique_ptr<Frame> pending_frame_;

  bool is_initialized_ = false;
  int img_num_ = 0;

  PoseMap tracker_poses_;
  std::queue<ros::Time> frame_time_queue_;
};

}} // namespace beam_models::camera_to_camera
