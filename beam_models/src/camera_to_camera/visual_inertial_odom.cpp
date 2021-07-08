#include <beam_models/camera_to_camera/visual_inertial_odom.h>
//#include <beam_common/extrinsics_lookup.h>
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
#include <beam_utils/time.h>
#include <fstream>
#include <iostream>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(beam_models::camera_to_camera::VisualInertialOdom,
                       fuse_core::SensorModel)

namespace beam_models { namespace camera_to_camera {

VisualInertialOdom::VisualInertialOdom()
    : fuse_core::AsyncSensorModel(1), device_id_(fuse_core::uuid::NIL) {}

void VisualInertialOdom::onInit() {
  Eigen::Matrix4d T_imu_cam;
  T_imu_cam << 0.0148655429818, -0.999880929698, 0.00414029679422,
      -0.0216401454975, 0.999557249008, 0.0149672133247, 0.025715529948,
      -0.064676986768, -0.0257744366974, 0.00375618835797, 0.999660727178,
      0.00981073058949, 0.0, 0.0, 0.0, 1.0;
  // beam_common::ExtrinsicsLookup::GetInstance().GetT_IMU_CAMERA(T_imu_cam);
  // std::cout << T_imu_cam << std::endl;
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  camera_params_.loadFromROS(private_node_handle_);
  imu_params_.loadFromROS(private_node_handle_);
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
      beam_calibration::CameraModel::Create(camera_params_.cam_intrinsics_path);
  visual_map_ = std::make_shared<VisualMap>(cam_model_, T_imu_cam, source_);
  /***********************************************************
   *              Initialize tracker variables               *
   ***********************************************************/
  std::shared_ptr<beam_cv::Descriptor> descriptor =
      std::make_shared<beam_cv::ORBDescriptor>();
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::GFTTDetector>(300);
  tracker_ = std::make_shared<beam_cv::KLTracker>(detector, descriptor,
                                                  camera_params_.window_size);
  /***********************************************************
   *                  Subscribe to topics                    *
   ***********************************************************/
  image_subscriber_ =
      private_node_handle_.subscribe(camera_params_.image_topic, 1000,
                                     &VisualInertialOdom::processImage, this);
  imu_subscriber_ = private_node_handle_.subscribe(
      imu_params_.imu_topic, 10000, &VisualInertialOdom::processIMU, this);
  path_subscriber_ = private_node_handle_.subscribe(
      camera_params_.init_path_topic, 1, &VisualInertialOdom::processInitPath,
      this);
  /***********************************************************
   *               Create initializer object                 *
   ***********************************************************/
  initializer_ =
      std::make_shared<beam_models::camera_to_camera::VIOInitializer>(
          cam_model_, tracker_, T_imu_cam, imu_params_.cov_gyro_noise,
          imu_params_.cov_accel_noise, imu_params_.cov_gyro_bias,
          imu_params_.cov_accel_bias);
  // temp
  init_path_pub_ = private_node_handle_.advertise<InitializedPathMsg>(
      camera_params_.init_path_topic, 1);
  BuildPath();
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
    tracker_->AddImage(ExtractImage(image_buffer_.front()), img_time);
    bool is_kf = IsKeyframe(img_time);
    if (!initializer_->Initialized() && is_kf) {
      // temp
      if (cur_kf_time_ > last_stamp_ && !set_once) {
        init_path_pub_.publish(init_path_);
        set_once = true;
      }
      if (initializer_->AddImage(img_time)) {
        ROS_INFO("Initialization Success.");
        // get the preintegration object
        imu_preint_ = initializer_->GetPreintegrator();
        // copy init graph and send to fuse optimizer
        SendInitializationGraph(initializer_->GetGraph());
      }
    } else if (initializer_->Initialized() && !is_kf) {
      // localize image and publish to internal frame initializer
    } else if (initializer_->Initialized() && is_kf) {
      // localize image and add to graph and publish to frame initializer
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
    if (!initializer_->Initialized()) {
      initializer_->AddIMU(imu_buffer_.front());
    } else {
      imu_preint_->AddToBuffer(imu_buffer_.front());
    }
    imu_buffer_.pop();
  }
}

void VisualInertialOdom::processInitPath(
    const InitializedPathMsg::ConstPtr& msg) {
  initializer_->SetPath(*msg);
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
  } else if ((img_time - cur_kf_time_).toSec() >= 0.8) {
    cur_kf_time_ = img_time;
    return true;
  }
  return false;
}

void VisualInertialOdom::SendInitializationGraph(
    const fuse_graphs::HashGraph& graph) {
  auto transaction = fuse_core::Transaction::make_shared();
  for (auto& var : init_graph.getVariables()) {
    fuse_variables::Position3D::SharedPtr landmark =
        fuse_variables::Position3D::make_shared();
    fuse_variables::Position3DStamped::SharedPtr position =
        fuse_variables::Position3DStamped::make_shared();
    fuse_variables::Orientation3DStamped::SharedPtr orientation =
        fuse_variables::Orientation3DStamped::make_shared();

    if (var.type() == landmark->type()) {
      *landmark = dynamic_cast<const fuse_variables::Position3D&>(var);
      visual_map_->AddLandmark(landmark, transaction);
    }
    if (var.type() == orientation->type()) {
      *orientation =
          dynamic_cast<const fuse_variables::Orientation3DStamped&>(var);
      visual_map_->AddOrientation(orientation, transaction);
    }
    if (var.type() == position->type()) {
      *position = dynamic_cast<const fuse_variables::Position3DStamped&>(var);
      visual_map_->AddPosition(position, transaction);
    }
  }
  for (auto& constraint : init_graph.getConstraints()) {
    transaction->addConstraint(std::move(constraint.clone()));
  }
  sendTransaction(transaction);
}

}} // namespace beam_models::camera_to_camera
