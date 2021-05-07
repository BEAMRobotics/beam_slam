#include <beam_models/camera_to_camera/vio_initializer.h>

#include <beam_models/camera_to_camera/opencvimage.h>

namespace beam_models { namespace camera_to_camera {

VIOInitializer::VIOInitializer(
    // std::shared_ptr<beam_cv::Tracker> tracker,
    std::shared_ptr<beam_calibration::CameraModel> cam_model,
    Eigen::Matrix4d T_body_cam, Eigen::Matrix4d T_body_imu,
    Eigen::Vector4d imu_intrinsics) {
  cam_model_ = cam_model;
  // tracker_ = tracker;
  config_ = std::make_shared<beam_models::camera_to_camera::CameraConfig>(
      cam_model, T_body_cam, T_body_imu, imu_intrinsics);
  // create image rectifier
  Eigen::Vector2i image_size;
  image_size << cam_model->GetHeight(), cam_model->GetWidth();
  rectifier_ = std::make_shared<beam_calibration::UndistortImages>(
      cam_model, image_size, image_size);
  // create initializer
  initializer_ = std::make_unique<VigInitializer>(config_);
}

bool VIOInitializer::AddImage(cv::Mat cur_img, ros::Time cur_time) {
  current_frame_time_ = cur_time;
  // create new frame
  pending_frame_ = this->CreateFrame();
  // build opencv image
  double t = cur_time.toSec();
  std::shared_ptr<OpenCvImage> img = std::make_shared<OpenCvImage>(cur_img);
  img->correct_distortion(rectifier_);
  img->preprocess();
  img->t = t;
  // set the frames image
  pending_frame_->image = img;
  initializer_->append_frame(std::move(pending_frame_));
  if (std::unique_ptr<SlidingWindow> sw = initializer_->init()) {
    sliding_window_.swap(sw);
    is_initialized = true;
    return true;
  } else {
    return false;
  }
}

void VIOInitializer::AddIMU(Eigen::Vector3d ang_vel, Eigen::Vector3d lin_accel,
                            ros::Time cur_time) {
  double t = cur_time.toSec();
  IMUData imu_data;
  imu_data.t = t;
  imu_data.w = ang_vel;
  imu_data.a = lin_accel;
  pending_frame_->preintegration.data.push_back(imu_data);
}

std::unique_ptr<Frame> VIOInitializer::CreateFrame() {
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

void VIOInitializer::GetPose(
    fuse_variables::Orientation3DStamped::SharedPtr orientation,
    fuse_variables::Position3DStamped::SharedPtr position) {}

// std::shared_ptr<beam_cv::Tracker> VIOInitializer::GetTracker() {
//   return tracker_;
// }

std::unordered_map<uint64_t, fuse_variables::Position3D::SharedPtr>
    VIOInitializer::GetLandmarks() {}

void VIOInitializer::GetBiases(Eigen::Vector3d& g, Eigen::Vector3d& bg,
                               Eigen::Vector3d& ba) {}

}} // namespace beam_models::camera_to_camera