#include <beam_models/camera_to_camera/vio_initializer.h>

namespace beam_models { namespace camera_to_camera {

VIOInitializer::VIOInitializer(
    std::shared_ptr<beam_calibration::CameraModel> cam_model,
    Eigen::Matrix4d T_body_cam, Eigen::Matrix4d T_body_imu,
    Eigen::Vector4d imu_intrinsics) {
  this->cam_model_ = cam_model;
  this->config_ = std::make_shared<beam_models::camera_to_camera::CameraConfig>(
      cam_model, T_body_cam, T_body_imu, imu_intrinsics);
  this->window_size_ = this->config_->init_map_frames();
  this->is_initialized_ = false;
  this->img_num_ = 0;
  // create image rectifier
  Eigen::Vector2i image_size;
  image_size << cam_model->GetHeight(), cam_model->GetWidth();
  this->rectifier_ = std::make_shared<beam_calibration::UndistortImages>(
      cam_model, image_size, image_size);
  // create initializer
  this->initializer_ = std::make_unique<VigInitializer>(config_);
  this->pending_frame_ = this->CreateFrame();
}

bool VIOInitializer::AddImage(cv::Mat cur_img, ros::Time cur_time) {
  std::shared_ptr<OpenCvImage> img = std::make_shared<OpenCvImage>(cur_img);
  img->correct_distortion(rectifier_);
  if (img_num_ == 0) { cv::imwrite("/home/jake/im.jpg", img->image); }
  img->t = cur_time.toSec();
  pending_frame_->image = img;

  initializer_->append_frame(std::move(this->pending_frame_));
  this->frame_time_queue_.push(cur_time);
  img_num_++;
  if (std::unique_ptr<SlidingWindow> sw = initializer_->init()) {
    sliding_window_.swap(sw);
    is_initialized_ = true;
    for (int i = 0; i < this->window_size_; i++) {
      ros::Time time = frame_time_queue_.front();
      // get frame pose from sliding window
      Frame* f = sliding_window_->get_frame(i);
      PoseState pose = f->pose;
      Eigen::Quaterniond q = pose.q;
      Eigen::Vector3d p = pose.p;
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
      T.block<3, 1>(0, 3) = p.transpose();
      // add frame pose to map
      tracker_poses_[time.toSec()] = T;
      // remove image from the queue
      frame_time_queue_.pop();
    }
    ROS_INFO("VIO Initialization Success.");
    return true;
  }
  this->pending_frame_ = this->CreateFrame();
  if (img_num_ > this->window_size_) { this->frame_time_queue_.pop(); }
  return false;
}

void VIOInitializer::AddIMU(Eigen::Vector3d ang_vel, Eigen::Vector3d lin_accel,
                            ros::Time cur_time) {
  IMUData imu_data;
  imu_data.t = cur_time.toSec();
  imu_data.w = ang_vel;
  imu_data.a = lin_accel;
  pending_frame_->preintegration.data.push_back(imu_data);
}

std::unordered_map<double, Eigen::Matrix4d> VIOInitializer::GetPoses() {
  if (is_initialized_) return tracker_poses_;
}

void VIOInitializer::GetBiases(Eigen::Vector3d& g, Eigen::Vector3d& bg,
                               Eigen::Vector3d& ba) {
  if (is_initialized_) {
    Frame* f = sliding_window_->get_frame(this->window_size_ - 1);
    MotionState m = f->motion;
    g = m.v;
    bg = m.bg;
    ba = m.ba;
  }
}

bool VIOInitializer::Initialized() {
  return is_initialized_;
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

}} // namespace beam_models::camera_to_camera
