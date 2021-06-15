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
  this->cam_model_ =
      beam_calibration::CameraModel::Create(params_.cam_intrinsics_path);
  this->visual_map_ = std::make_shared<VisualMap>(this->cam_model_, T_imu_cam_);
  /***********************************************************
   *              Initialize tracker variables               *
   ***********************************************************/
  std::shared_ptr<beam_cv::Matcher> matcher =
      std::make_shared<beam_cv::BFMatcher>(cv::NORM_HAMMING, false, false);
  std::shared_ptr<beam_cv::Descriptor> descriptor =
      std::make_shared<beam_cv::ORBDescriptor>();
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::FASTDetector>(1000);
  this->tracker_ = std::make_shared<beam_cv::Tracker>(
      detector, descriptor, matcher, params_.window_size);
  /***********************************************************
   *                  Subscribe to topics                    *
   ***********************************************************/
  image_subscriber_ = private_node_handle_.subscribe(
      params_.image_topic, 1000, &VisualInertialOdom::processImage, this);
  imu_subscriber_ = private_node_handle_.subscribe(
      params_.imu_topic, 10000, &VisualInertialOdom::processIMU, this);
  /***********************************************************
   *               Create initializer object                 *
   ***********************************************************/
  Eigen::Vector4d imu_intrinsics(params_.imu_intrinsics.data());
  initializer_ =
      std::make_shared<VIOInitializer>(cam_model_, T_imu_cam_, imu_intrinsics);
}

void VisualInertialOdom::processImage(const sensor_msgs::Image::ConstPtr& msg) {
  // get message info
  image_buffer_.push(*msg);
  sensor_msgs::Image img_msg = image_buffer_.front();
  ros::Time img_time = img_msg.header.stamp;
  /**************************************************************************
   *              Add IMU messages to buffer or initializer                 *
   **************************************************************************/
  while (imu_buffer_.front().header.stamp <= img_time && !imu_buffer_.empty()) {
    sensor_msgs::Imu imu_msg = imu_buffer_.front();
    ros::Time imu_time = imu_msg.header.stamp;
    if (!initializer_->Initialized()) {
      temp_imu_buffer_.push(imu_msg);
      Eigen::Vector3d ang_vel{imu_msg.angular_velocity.x,
                              imu_msg.angular_velocity.y,
                              imu_msg.angular_velocity.z};
      Eigen::Vector3d lin_accel{imu_msg.linear_acceleration.x,
                                imu_msg.linear_acceleration.y,
                                imu_msg.linear_acceleration.z};
      initializer_->AddIMU(ang_vel, lin_accel, imu_time);
    } else {
      imu_preint_->PopulateBuffer(imu_msg);
    }
    imu_buffer_.pop();
  }
  /**************************************************************************
   *                    Add Image to map or initializer                     *
   **************************************************************************/
  if (!imu_buffer_.empty()) {
    img_num_++;
    cv::Mat image = this->extractImage(img_msg);
    if (!initializer_->Initialized()) {
      if (initializer_->AddImage(image, img_time)) {
        // this transaction adds the first pose, adds the initial map points
        // and initializes the preintegrator object
        auto init_transaction = this->initMap();
        sendTransaction(init_transaction);
        // register the current frame against the map
        this->tracker_->AddImage(image, img_time);
        auto frame_transaction = this->registerFrame(image, img_time);
        sendTransaction(frame_transaction);
      } else {
        this->tracker_->AddImage(image, img_time);
        std::queue<sensor_msgs::Imu>().swap(temp_imu_buffer_);
      }
    } else {
      this->tracker_->AddImage(image, img_time);
      // auto frame_transaction = this->registerFrame(image, img_time);
      // sendTransaction(frame_transaction);
    }
    image_buffer_.pop();
  }
}

void VisualInertialOdom::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  // indiscriminantly push imu messages onto its buffer
  imu_buffer_.push(*msg);
}

void VisualInertialOdom::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  this->visual_map_->updateGraph(graph);
}

void VisualInertialOdom::onStop() {}

std::shared_ptr<fuse_core::Transaction> VisualInertialOdom::initMap() {
  auto transaction = fuse_core::Transaction::make_shared();
  // Get first and last poses estimated by initializer
  beam_models::camera_to_camera::PoseMap poses = initializer_->GetPoses();
  ros::Time first_kf_time;
  size_t i = 0;
  size_t poses_size = poses.size();
  Eigen::Matrix4d T_world_kf1, T_world_kf2;
  for (auto& pose : poses) {
    if (i == 0) {
      first_kf_time.fromNSec(pose.first);
      T_world_kf1 = pose.second;
    } else if (i == poses.size() - 1) {
      cur_kf_time_.fromNSec(pose.first);
      T_world_kf2 = pose.second;
    }
    i++;
  }
  // recompute relative orientation between first and last using more robust
  // estimator
  std::vector<uint64_t> cur_kf_ids =
      this->tracker_->GetLandmarkIDsInImage(cur_kf_time_);
  std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pL_v, pR_v;
  for (auto& id : cur_kf_ids) {
    try {
      Eigen::Vector2i pL = this->tracker_->Get(first_kf_time, id).cast<int>();
      Eigen::Vector2i pR = this->tracker_->Get(cur_kf_time_, id).cast<int>();
      pL_v.push_back(pL);
      pR_v.push_back(pR);
    } catch (const std::out_of_range& oor) {}
  }
  Eigen::Matrix3d R_frame2_frame1 =
      beam_cv::RelativePoseEstimator::RANSACEstimator(
          this->cam_model_, this->cam_model_, pL_v, pR_v,
          beam_cv::EstimatorMethod::SEVENPOINT)
          .value()
          .block<3, 3>(0, 0);
  T_world_kf2.block<3, 3>(0, 0) =
      T_world_kf1.block<3, 3>(0, 0) * R_frame2_frame1.inverse();
  // add poses to the map
  this->visual_map_->addPose(T_world_kf1, first_kf_time, transaction);
  this->visual_map_->addPose(T_world_kf2, cur_kf_time_, transaction);
  std::cout << "Initialized Keyframe Pose: " << std::endl;
  std::cout << T_world_kf2 << std::endl;
  // pcl::PointCloud<pcl::PointXYZRGB> frames1, frames2;
  // frames1 = beam::AddFrameToCloud(frames1, T_world_kf1, 0.001, 0.1);
  // frames1 = beam::AddFrameToCloud(frames1, T_world_kf2, 0.001, 0.1);
  /******************************************************
   *           Add landmarks and constraints            *
   ******************************************************/
  int num_lm = 0;
  for (auto& id : cur_kf_ids) {
    try {
      Eigen::Vector2d pL = this->tracker_->Get(first_kf_time, id);
      Eigen::Vector2d pR = this->tracker_->Get(cur_kf_time_, id);
      beam::opt<Eigen::Vector3d> point =
          this->triangulate(tracker_->GetTrack(id));
      if (point.has_value()) {
        num_lm++;
        this->visual_map_->addLandmark(point.value(), id, transaction);
        this->visual_map_->addConstraint(cur_kf_time_, id, pL, transaction);
        this->visual_map_->addConstraint(first_kf_time, id, pR, transaction);
      }
    } catch (const std::out_of_range& oor) {}
  }
  ROS_INFO("Initialized Map Points: %d", num_lm);
  this->initIMU();
  return transaction;
}

std::shared_ptr<fuse_core::Transaction>
    VisualInertialOdom::registerFrame(const cv::Mat& image,
                                      const ros::Time& img_time) {
  auto transaction = fuse_core::Transaction::make_shared();
  // [1] get landmark ids in current image
  std::vector<uint64_t> cur_img_ids =
      this->tracker_->GetLandmarkIDsInImage(img_time);
  // [2] retrieve positions of the landmarks in the image
  std::set<uint64_t> triangulated_ids_set, nontriangulated_ids_set;
  std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam_cv::AlignVec3d> points;
  for (auto& id : cur_img_ids) {
    Eigen::Vector2i pixel = tracker_->Get(img_time, id).cast<int>();
    fuse_variables::Position3D::SharedPtr lm =
        this->visual_map_->getLandmark(id);
    if (lm) {
      Eigen::Vector3d point(lm->data());
      pixels.push_back(pixel);
      points.push_back(point);
      triangulated_ids_set.insert(id);
    } else {
      nontriangulated_ids_set.insert(id);
    }
  }
  // [3] Get IMU Pose estimate of frame
  Eigen::Matrix4d T_world_frame = imu_preint_->GetPose(img_time) * T_imu_cam_;
  std::cout << "\n------------------" << std::endl;
  std::cout << "IMU Estimate: " << std::endl;
  std::cout << T_world_frame << std::endl;

  Eigen::Matrix4d pose = beam_cv::AbsolutePoseEstimator::RANSACEstimator(
      this->cam_model_, pixels, points);
  std::cout << "PnP Estimate: " << std::endl;
  std::cout << pose << std::endl;

  std::string report;
  Eigen::Matrix4d T_world_cam = pose_refiner_->RefinePose(
      T_world_frame, cam_model_, pixels, points, report);
  std::cout << "Refined Estimate: " << std::endl;
  std::cout << T_world_cam << std::endl;
  // // [5] Perform keyframe processing
  // if (img_num_ % 5 == 0) {
  //   keyframes_.push_back(img_time.toNSec());
  //   // TODO: Refine keyframe pose with inlier landmarks if possible
  //   // [5.1] Add current pose to the graph
  //   this->visual_map_->addPose(T_world_frame, img_time, transaction);
  //   // [5.2] Add constraints to currently tracked landmarks
  //   for (auto& id : triangulated_ids_set) {
  //     this->visual_map_->addConstraint(
  //         img_time, id, tracker_->Get(img_time, id), transaction);
  //   }
  //   int num_lm = 0;
  //   // [5.3] Triangulate new features and add constraints
  //   for (auto& id : nontriangulated_ids_set) {
  //     // get observation in current frame
  //     Eigen::Vector2d cur_frame_pix = tracker_->Get(img_time, id);
  //     // check if id is visible in previous keyframe
  //     Eigen::Vector2d cur_kf_pix;
  //     try {
  //       cur_kf_pix = tracker_->Get(cur_kf_time_, id);
  //     } catch (const std::out_of_range& oor) { continue; }
  //     // if it is in previous keyframe, triangulate the point
  //     beam::opt<Eigen::Vector3d> point =
  //         this->triangulate(tracker_->GetTrack(id));
  //     if (point.has_value()) {
  //       num_lm++;
  //       this->visual_map_->addLandmark(point.value(), id, transaction);
  //       this->visual_map_->addConstraint(img_time, id, cur_frame_pix,
  //                                        transaction);
  //       this->visual_map_->addConstraint(cur_kf_time_, id, cur_kf_pix,
  //                                        transaction);
  //     }
  //   }
  //   std::cout << "Visible points: " << triangulated_ids_set.size()
  //             << " Added points: " << num_lm << std::endl;
  //   cur_kf_time_ = img_time;
  //   //[6.4] Create imu_preint constraint between the current frame and last
  //   kf
  //   // this->addIMUConstraint(img_time, transaction);
  // } else {
  //   non_keyframe_poses_[img_time.toNSec()] = T_world_frame;
  //   for (auto& p : non_keyframe_poses_) {
  //     if (non_keyframe_poses_.size() > 20) {
  //       non_keyframe_poses_.erase(p.first);
  //       break;
  //     }
  //   }
  // }
  return transaction;
}

beam::opt<Eigen::Vector3d>
    VisualInertialOdom::triangulate(beam_cv::FeatureTrack track) {
  if (track.size() >= 2) {
    std::vector<Eigen::Matrix4d, beam_cv::AlignMat4d> T_cam_world_v;
    std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
    for (auto& measurement : track) {
      ros::Time stamp = measurement.time_point;
      beam::opt<Eigen::Matrix4d> T = this->visual_map_->getPose(stamp);
      // check if the pose is in the graph (keyframe)
      if (T.has_value()) {
        pixels.push_back(measurement.value.cast<int>());
        T_cam_world_v.push_back(T.value());
      }
    }
    beam::opt<Eigen::Vector3d> point = beam_cv::Triangulation::TriangulatePoint(
        this->cam_model_, T_cam_world_v, pixels, 10.0);
    return point;
  }
  return {};
}

cv::Mat VisualInertialOdom::extractImage(const sensor_msgs::Image& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  return cv_ptr->image;
}

void VisualInertialOdom::initIMU() {
  Eigen::Vector3d v, bg, ba;
  initializer_->GetBiases(v, bg, ba);
  fuse_variables::VelocityLinear3DStamped::SharedPtr velocity =
      fuse_variables::VelocityLinear3DStamped::make_shared(cur_kf_time_);
  velocity->x() = v[0];
  velocity->y() = v[1];
  velocity->z() = v[2];
  beam_models::frame_to_frame::ImuPreintegration::Params imu_params;
  imu_preint_ =
      std::make_shared<beam_models::frame_to_frame::ImuPreintegration>(
          imu_params, bg, ba);
  // push imu messages stored between last frame in vio initializer
  // (cur_kf_time) and the current frame (img_time)
  while (temp_imu_buffer_.size() > 0) {
    imu_preint_->PopulateBuffer(temp_imu_buffer_.front());
    temp_imu_buffer_.pop();
  }
  imu_preint_->SetStart(cur_kf_time_,
                        this->visual_map_->getOrientation(cur_kf_time_),
                        this->visual_map_->getPosition(cur_kf_time_), velocity);
}

void VisualInertialOdom::addIMUConstraint(
    ros::Time keyframe_time,
    std::shared_ptr<fuse_core::Transaction> transaction) {
  fuse_variables::Orientation3DStamped::SharedPtr img_orientation =
      this->visual_map_->getOrientation(keyframe_time);
  fuse_variables::Position3DStamped::SharedPtr img_position =
      this->visual_map_->getPosition(keyframe_time);
  beam_constraints::frame_to_frame::ImuState3DStampedTransaction imu_trans =
      imu_preint_->RegisterNewImuPreintegratedFactor(
          keyframe_time, img_orientation, img_position);
  transaction->merge(*imu_trans.GetTransaction());
}
}} // namespace beam_models::camera_to_camera
