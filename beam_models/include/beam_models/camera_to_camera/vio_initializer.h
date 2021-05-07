#pragma once
// libbeam
#include <beam_calibration/CameraModels.h>
#include <beam_calibration/ConvertCameraModel.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_cv/tracker/Tracker.h>
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
#include <slamtools/vig_initializer.h>

namespace beam_models { namespace camera_to_camera {

class VIOInitializer {
public:
  VIOInitializer() = default;

  VIOInitializer(std::shared_ptr<beam_cv::Tracker> tracker,
                 std::shared_ptr<beam_calibration::CameraModel> cam_model,
                 Eigen::Matrix4d T_body_cam, Eigen::Matrix4d T_body_imu,
                 Eigen::Vector4d imu_intrinsics) {
    this->cam_model_ = cam_model;
    this->tracker_ = tracker;
    this->config_ =
        std::make_shared<beam_models::camera_to_camera::CameraConfig>(
            cam_model, T_body_cam, T_body_imu, imu_intrinsics);
    // create image rectifier
    Eigen::Vector2i image_size;
    image_size << cam_model->GetHeight(), cam_model->GetWidth();
    this->rectifier_ = std::make_shared<beam_calibration::UndistortImages>(
        cam_model, image_size, image_size);
    // create initializer
    this->initializer_ = std::make_unique<VigInitializer>(config_);
    this->pending_frame_ = this->CreateFrame();
  }

  bool AddImage(cv::Mat cur_img, ros::Time cur_time) {
    this->current_frame_time_ = cur_time;
    // build opencv image
    double t = cur_time.toSec();
    std::shared_ptr<OpenCvImage> img = std::make_shared<OpenCvImage>(cur_img);
    img->correct_distortion(rectifier_);
    img->preprocess();
    img->t = t;
    // set the frames image
    pending_frame_->image = img;
    initializer_->append_frame(std::move(this->pending_frame_));
    if (std::unique_ptr<SlidingWindow> sw = initializer_->init()) {
      sliding_window_.swap(sw);
      std::cout << "SUCCESSFULLY INITIALIZED" << std::endl;
      return true;
    }
    this->pending_frame_ = this->CreateFrame();
    return false;
  }

  void AddIMU(Eigen::Vector3d ang_vel, Eigen::Vector3d lin_accel,
              ros::Time cur_time) {
    double t = cur_time.toSec();
    IMUData imu_data;
    imu_data.t = t;
    imu_data.w = ang_vel;
    imu_data.a = lin_accel;
    pending_frame_->preintegration.data.push_back(imu_data);
  }

  void GetPose(fuse_variables::Orientation3DStamped::SharedPtr orientation,
               fuse_variables::Position3DStamped::SharedPtr position) {}

  std::shared_ptr<beam_cv::Tracker> GetTracker() { return tracker_; }

  std::unordered_map<uint64_t, fuse_variables::Position3D::SharedPtr>
      GetLandmarks() {}

  void GetBiases(Eigen::Vector3d& g, Eigen::Vector3d& bg, Eigen::Vector3d& ba) {
  }

private:
  std::unique_ptr<Frame> CreateFrame() {
    std::unique_ptr<Frame> frame = std::make_unique<Frame>();
    frame->K = config_->camera_intrinsic();
    frame->sqrt_inv_cov =
        frame->K.block<2, 2>(0, 0) / ::sqrt(config_->keypoint_pixel_error());
    frame->camera.q_cs = config_->camera_to_center_rotation();
    frame->camera.p_cs = config_->camera_to_center_translation();
    frame->imu.q_cs = config_->imu_to_center_rotation();
    frame->imu.p_cs = config_->imu_to_center_translation();
    frame->preintegration.cov_w = config_->imu_gyro_white_noise();
    frame->preintegration.cov_a = config_->imu_accel_white_noise();
    frame->preintegration.cov_bg = config_->imu_gyro_random_walk();
    frame->preintegration.cov_ba = config_->imu_accel_random_walk();
    return frame;
  }

protected:
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<beam_calibration::UndistortImages> rectifier_;

  ros::Time current_frame_time_;

  std::unique_ptr<SlidingWindow> sliding_window_;
  std::unique_ptr<VigInitializer> initializer_;

  std::shared_ptr<beam_models::camera_to_camera::CameraConfig> config_;

  std::unique_ptr<Frame> pending_frame_;
  bool is_initialized = false;
};

}} // namespace beam_models::camera_to_camera
