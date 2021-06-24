#include <beam_models/camera_to_camera/visual_inertial_odom.h>
// fuse
#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>
// other
#include <cv_bridge/cv_bridge.h>
// libbeam
#include <beam_cv/Utils.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_cv/matchers/Matchers.h>
#include <beam_utils/pointclouds.h>
#include <beam_utils/time.h>
#include <fstream>
#include <iostream>
#include <nav_msgs/Path.h>
#include <pcl/io/pcd_io.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(beam_models::camera_to_camera::VisualInertialOdom,
                       fuse_core::SensorModel)

namespace beam_models { namespace camera_to_camera {

VisualInertialOdom::VisualInertialOdom()
    : fuse_core::AsyncSensorModel(1), device_id_(fuse_core::uuid::NIL) {}

void VisualInertialOdom::onInit() {
  T_imu_cam_ << 0.0148655429818, -0.999880929698, 0.00414029679422,
      -0.0216401454975, 0.999557249008, 0.0149672133247, 0.025715529948,
      -0.064676986768, -0.0257744366974, 0.00375618835797, 0.999660727178,
      0.00981073058949, 0.0, 0.0, 0.0, 1.0;
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  params_.loadFromROS(private_node_handle_);
  /***********************************************************
   *       Initialize pose refiner object with params        *
   ***********************************************************/
  ceres::Solver::Options pose_refinement_options;
  pose_refinement_options.minimizer_progress_to_stdout = false;
  pose_refinement_options.max_num_iterations = 10;
  pose_refinement_options.max_solver_time_in_seconds = 1e-2;
  pose_refinement_options.function_tolerance = 1e-4;
  pose_refinement_options.gradient_tolerance = 1e-6;
  pose_refinement_options.parameter_tolerance = 1e-4;
  pose_refinement_options.linear_solver_type = ceres::SPARSE_SCHUR;
  pose_refinement_options.preconditioner_type = ceres::SCHUR_JACOBI;
  pose_refiner_ =
      std::make_shared<beam_cv::PoseRefinement>(pose_refinement_options);
  /***********************************************************
   *        Load camera model and Create Map object          *
   ***********************************************************/
  cam_model_ =
      beam_calibration::CameraModel::Create(params_.cam_intrinsics_path);
  visual_map_ = std::make_shared<VisualMap>(cam_model_, T_imu_cam_, source_);
  /***********************************************************
   *              Initialize tracker variables               *
   ***********************************************************/
  std::shared_ptr<beam_cv::Matcher> matcher =
      std::make_shared<beam_cv::BFMatcher>(cv::NORM_HAMMING, false, false);
  std::shared_ptr<beam_cv::Descriptor> descriptor =
      std::make_shared<beam_cv::ORBDescriptor>();
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::FASTDetector>(1000);
  tracker_ = std::make_shared<beam_cv::Tracker>(detector, descriptor, matcher,
                                                params_.window_size);
  /***********************************************************
   *                  Subscribe to topics                    *
   ***********************************************************/
  image_subscriber_ = private_node_handle_.subscribe(
      params_.image_topic, 1000, &VisualInertialOdom::processImage, this);
  imu_subscriber_ = private_node_handle_.subscribe(
      params_.imu_topic, 10000, &VisualInertialOdom::processIMU, this);
  path_subscriber_ = private_node_handle_.subscribe(
      params_.lidar_init_path_topic, 1, &VisualInertialOdom::processPath, this);
}

void VisualInertialOdom::processImage(const sensor_msgs::Image::ConstPtr& msg) {
  // push image onto buffer
  image_buffer_.push(*msg);
  // get current imu and image timestamps
  ros::Time imu_time = imu_buffer_.front().header.stamp;
  ros::Time img_time = image_buffer_.front().header.stamp;
  /**************************************************************************
   *                    Add Image to map or initializer                     *
   **************************************************************************/
  if (imu_time > img_time && !imu_buffer_.empty()) {
    cv::Mat image = ExtractImage(image_buffer_.front());
    tracker_->AddImage(image, img_time);
    if (IsKeyframe(img_time)) {
      // handle keyframe
    } else {
      // handle non-keyframe
    }
    image_buffer_.pop();
  }
}

void VisualInertialOdom::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  // push imu message onto buffer
  imu_buffer_.push(*msg);
  // get current image timestamp
  ros::Time img_time = image_buffer_.front().header.stamp;
  /**************************************************************************
   *          Add IMU messages to preintegrator or initializer              *
   **************************************************************************/
  while (imu_buffer_.front().header.stamp <= img_time && !imu_buffer_.empty()) {
    // handle imu messgae
    imu_buffer_.pop();
  }
}

void VisualInertialOdom::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  visual_map_->UpdateGraph(graph);
}

void VisualInertialOdom::onStop() {}

cv::Mat VisualInertialOdom::ExtractImage(const sensor_msgs::Image& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  return cv_ptr->image;
}

bool VisualInertialOdom::IsKeyframe(ros::Time img_time) {
  if (cur_kf_time_ == ros::Time(0)) {
    cur_kf_time_ = img_time;
    return true;
  }
  std::vector<uint64_t> cur_img_ids = tracker_->GetLandmarkIDsInImage(img_time);
  int num_matches = 0;
  for (auto& id : cur_img_ids) {
    try {
      tracker_->Get(cur_kf_time_, id);
      tracker_->Get(img_time, id);
      num_matches++;
    } catch (const std::out_of_range& oor) {}
  }
  if (num_matches <= 200) {
    cur_kf_time_ = img_time;
    return true;
  } else {
    return false;
  }
}

}} // namespace beam_models::camera_to_camera
