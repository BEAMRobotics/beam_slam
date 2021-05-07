#include <beam_models/camera_to_camera/visual_odom.h>
// fuse
#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>
// other
#include <cv_bridge/cv_bridge.h>
// libbeam
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_cv/matchers/Matchers.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(beam_models::camera_to_camera::VisualOdom,
                       fuse_core::SensorModel)

namespace beam_models { namespace camera_to_camera {

VisualOdom::VisualOdom()
    : fuse_core::AsyncSensorModel(1), device_id_(fuse_core::uuid::NIL) {}

void VisualOdom::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  params_.loadFromROS(private_node_handle_);
  // create camera models
  std::shared_ptr<beam_calibration::CameraModel> cam_model_ =
      beam_calibration::CameraModel::Create(params_.cam_intrinsics_path);
  // create matcher
  std::shared_ptr<beam_cv::Matcher> matcher =
      std::make_shared<beam_cv::FLANNMatcher>(beam_cv::FLANN::KDTree, 0.8, true,
                                              true, cv::FM_RANSAC, 1);
  // create descriptor
  std::shared_ptr<beam_cv::Descriptor> descriptor =
      std::make_shared<beam_cv::ORBDescriptor>();
  // create detector
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::FASTDetector>(300);
  // subscribe to image topic
  image_subscriber_ = private_node_handle_.subscribe(
      params_.image_topic, 100, &VisualOdom::processImage, this);
  // subscribe to imu topic
  imu_subscriber_ = private_node_handle_.subscribe(
      params_.imu_topic, 1000, &VisualOdom::processIMU, this);

  std::cout << params_.image_topic << std::endl;
  std::cout << params_.imu_topic << std::endl;
  // make initializer
  // Eigen::Matrix4d T_body_cam;
  // Eigen::Matrix4d T_body_imu;
  // Eigen::Vector4d imu_intrinsics;
  // initializer_ =
  //     std::make_shared<beam_models::camera_to_camera::VIOInitializer>(
  //         tracker_, cam_model_, T_body_cam, T_body_imu, imu_intrinsics);
}

void VisualOdom::processImage(const sensor_msgs::Image::ConstPtr& msg) {
  // get message info
  image_buffer_.push(*msg);
  sensor_msgs::Image img_msg = image_buffer_.front();
  ros::Time img_time = img_msg.header.stamp;
  // push imu messages for the previous front of the images
  while (imu_buffer_.front().header.stamp < img_time && !imu_buffer_.empty()) {
    if (img_num_ > 0) {
      sensor_msgs::Imu imu_msg = imu_buffer_.front();
      ros::Time imu_time = imu_msg.header.stamp;
      std::cout << "IMU msg push: " << imu_time << std::endl;
      Eigen::Vector3d ang_vel{imu_msg.angular_velocity.x,
                              imu_msg.angular_velocity.y,
                              imu_msg.angular_velocity.z};
      Eigen::Vector3d lin_accel{imu_msg.linear_acceleration.x,
                                imu_msg.linear_acceleration.y,
                                imu_msg.linear_acceleration.z};
      // process imu data
      if (init_mode) {
        // initializer_->AddIMU(ang_vel, lin_accel, imu_time);
      } else {
        // preintegrator.PopulateBuffer(msg);
      }
    }
    imu_buffer_.pop();
  }
  if (!imu_buffer_.empty()) {
    img_num_++;
    std::cout << "IMAGE msg push: " << img_time << std::endl;
    // process image data
    if (init_mode) {
      // init_mode = initializer_->AddImage(this->extractImage(img_msg),
      // img_time);
      // preintegrator.SetStart(img_time);
    } else {
      // this->RegisterFrame(cur_img, cur_time)
    }
    image_buffer_.pop();
  }
}

void VisualOdom::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_buffer_.push(*msg);
}

void VisualOdom::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {}

void VisualOdom::onStop() {}

cv::Mat VisualOdom::extractImage(const sensor_msgs::Image& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  return cv_ptr->image;
}

}} // namespace beam_models::camera_to_camera
