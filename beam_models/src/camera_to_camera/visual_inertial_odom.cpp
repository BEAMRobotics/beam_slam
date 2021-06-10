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
  pose_refinement_options.function_tolerance = 1e-6;
  pose_refinement_options.gradient_tolerance = 1e-8;
  pose_refinement_options.parameter_tolerance = 1e-6;
  pose_refinement_options.trust_region_strategy_type = ceres::DOGLEG;
  pose_refinement_options.linear_solver_type = ceres::DENSE_SCHUR;
  // pose_refinement_options.preconditioner_type = ceres::SCHUR_JACOBI;
  pose_refiner_ =
      std::make_shared<beam_cv::PoseRefinement>(pose_refinement_options);
  /***********************************************************
   *        Load camera model and Create Map object          *
   ***********************************************************/
  this->cam_model_ =
      beam_calibration::CameraModel::Create(params_.cam_intrinsics_path);
  this->visual_map_ =
      std::make_shared<VisualMap>(this->source_, this->cam_model_, T_imu_cam_);
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
    cv::Mat image = this->extractImage(img_msg);
    if (!initializer_->Initialized()) {
      kf_images_[img_time.toNSec()] = image;
      if (initializer_->AddImage(image, img_time)) {
        // this transaction adds the first pose, adds the initial map points
        // and initializes the preintegrator object
        auto init_transaction = this->initMap();
        sendTransaction(init_transaction);
        // register the current frame against the map
        // tracker_->AddImage(image, img_time);
        // auto frame_transaction = this->registerFrame(image, img_time);
        // sendTransaction(frame_transaction);
      } else {
        tracker_->AddImage(image, img_time);
        std::queue<sensor_msgs::Imu>().swap(temp_imu_buffer_);
      }
    } else {
      // tracker_->AddImage(image, img_time);
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
}

void VisualInertialOdom::onStop() {}

std::shared_ptr<fuse_core::Transaction> VisualInertialOdom::initMap() {
  auto transaction = fuse_core::Transaction::make_shared();
  /******************************************************
   *           Add initializer poses to map             *
   ******************************************************/
  std::map<unsigned long, Eigen::Matrix4d> poses = initializer_->GetPoses();
  ros::Time first_kf_time;
  size_t i = 0;
  size_t poses_size = poses.size();
  for (auto& pose : poses) {
    if (i == 0) {
      first_kf_time.fromNSec(pose.first);
      this->visual_map_->addPose(pose.second, first_kf_time, transaction);
      keyframes_.push_back(pose.first);
    } else if (i == poses.size() - 1) {
      cur_kf_time_.fromNSec(pose.first);
      this->visual_map_->addPose(pose.second, cur_kf_time_, transaction);
      keyframes_.push_back(pose.first);
    } else {
      ros::Time this_frame_time;
      frame_poses_[first_kf_time.toNSec()][pose.first] = pose.second;
    }
    i++;
  }
  // std::cout << first_kf_time << "    Pose: \n"
  //           << this->visual_map_->getPose(first_kf_time).value().inverse()
  //           << std::endl;
  // std::cout << cur_kf_time_ << "    Pose: \n"
  //           << this->visual_map_->getPose(cur_kf_time_).value().inverse()
  //           << std::endl;

  // pcl::PointCloud<pcl::PointXYZRGB> frames;
  // frames = beam::AddFrameToCloud(
  //     frames, this->visual_map_->getPose(first_kf_time).value().inverse(),
  //     0.001, 0.1);
  // frames = beam::AddFrameToCloud(
  //     frames, this->visual_map_->getPose(cur_kf_time_).value().inverse(),
  //     0.001, 0.1);
  // pcl::io::savePCDFileBinary("/home/jake/frames.pcd", frames);
  // cv::imwrite("/home/jake/kf1.jpg", kf_images_[first_kf_time.toNSec()]);
  // cv::imwrite("/home/jake/kf2.jpg", kf_images_[cur_kf_time_.toNSec()]);
  /******************************************************
   *           Add landmarks and constraints            *
   ******************************************************/
  // triangulate all points in all
  pcl::PointCloud<pcl::PointXYZ> points_cloud;
  int num_lm = 0;
  std::vector<uint64_t> lm_ids = tracker_->GetLandmarkIDsInImage(cur_kf_time_);
  for (int i = 0; i < lm_ids.size(); i++) {
    uint64_t id = lm_ids[i];
    // get landmark track
    beam_cv::FeatureTrack track = tracker_->GetTrack(id);
    // get pixel of this landmark in the current image
    Eigen::Vector2d pix1 = tracker_->Get(cur_kf_time_, id);
    Eigen::Vector2d pix2;
    try {
      pix2 = tracker_->Get(first_kf_time, id);
    } catch (const std::out_of_range& oor) { continue; }
    // triangulate the track and add to transaction
    beam::opt<Eigen::Vector3d> point = this->triangulate(track);
    if (point.has_value()) {
      // check if resulting point is an inlier
      // pcl::PointXYZ p;
      // p.x = point.value()[0];
      // p.y = point.value()[1];
      // p.z = point.value()[2];
      // points_cloud.push_back(p);
      num_lm++;
      this->visual_map_->addLandmark(point.value(), id, transaction);
      this->visual_map_->addConstraint(cur_kf_time_, id, pix1, transaction);
      this->visual_map_->addConstraint(first_kf_time, id, pix2, transaction);
    }
  }
  // pcl::io::savePCDFileBinary("/home/jake/points.pcd", points_cloud);
  ROS_INFO("Initialized Map Points: %d", num_lm);
  this->initIMU();
  return transaction;
}

std::shared_ptr<fuse_core::Transaction>
    VisualInertialOdom::registerFrame(const cv::Mat& image,
                                      const ros::Time& img_time) {
  // std::cout << "\nImage: " << img_time << std::endl;
  auto transaction = fuse_core::Transaction::make_shared();
  // [1] get landmark ids in current image
  std::vector<uint64_t> cur_img_ids =
      this->tracker_->GetLandmarkIDsInImage(img_time);
  // [2] retrieve positions of the landmarks in the image
  std::set<uint64_t> triangulated_ids_set, nontriangulated_ids_set;
  for (auto& id : cur_img_ids) {
    fuse_variables::Position3D::SharedPtr lm =
        this->visual_map_->getLandmark(id);
    if (lm) {
      triangulated_ids_set.insert(id);
    } else {
      nontriangulated_ids_set.insert(id);
    }
  }
  // [3] Get IMU Pose estimate of frame
  Eigen::Matrix4d T_world_cam_est =
      imu_preint_->GetPose(img_time) * T_imu_cam_.inverse();
  // [4] Add to keyframe to keyframe window
  frame_poses_[keyframes_.front()][img_time.toNSec()] = T_world_cam_est;
  // [6] Perform keyframe processing
  if (triangulated_ids_set.size() < 30) {
    ros::Time cur_kf_time;
    cur_kf_time.fromNSec(keyframes_.front());
    // TODO: Refine keyframe pose with inlier landmarks if possible
    // [6.1] Add current pose to the graph
    this->visual_map_->addPose(T_world_cam_est, img_time, transaction);
    // [6.2] Add constraints to currently tracked landmarks
    for (auto& id : triangulated_ids_set) {
      this->visual_map_->addConstraint(
          img_time, id, tracker_->Get(img_time, id), transaction);
    }
    int num_triangulated = 0;
    // [6.3] Triangulate new features and add constraints
    for (auto& id : nontriangulated_ids_set) {
      // check if id is visible in previous keyframe
      Eigen::Vector2d prev_kf_measurement;
      try {
        prev_kf_measurement = tracker_->Get(cur_kf_time, id)
      } catch (const std::out_of_range& oor) { continue; }
      // if it is in previous keyframe, triangulate the point
      beam::opt<Eigen::Vector3d> point =
          this->triangulate(tracker_->GetTrack(id));
      if (point.has_value()) {
        // check if the resulting point is an inlier
        num_triangulated++;
        this->visual_map_->addLandmark(point.value(), id, transaction);
        this->visual_map_->addConstraint(
            img_time, id, tracker_->Get(img_time, id), transaction);
        try {
          this->visual_map_->addConstraint(
              cur_kf_time, id, tracker_->Get(cur_kf_time, id), transaction);
        } catch (const std::out_of_range& oor) {}
      }
    }
    // std::cout << "New landmark points: " << num_triangulated << std::endl;
    //[6.4] Create imu_preint constraint between the current frame and last kf
    fuse_variables::Orientation3DStamped::SharedPtr img_orientation =
        this->visual_map_->getOrientation(img_time);
    fuse_variables::Position3DStamped::SharedPtr img_position =
        this->visual_map_->getPosition(img_time);
    beam_constraints::frame_to_frame::ImuState3DStampedTransaction imu_trans =
        imu_preint_->RegisterNewImuPreintegratedFactor(
            img_time, img_orientation, img_position);
    transaction->merge(*imu_trans.GetTransaction());
    keyframes_.push_back(img_time.toNSec());
  }

  return transaction;
}

beam::opt<Eigen::Vector3d>
    VisualInertialOdom::triangulate(beam_cv::FeatureTrack track) {
  // 1. check if track length is seen in current frame and previous keyframe
  // 2. get the t
  if (track.size() >= 2) {
    std::vector<Eigen::Matrix4d> T_cam_world_v;
    std::vector<Eigen::Vector2i> pixels;
    for (auto& measurement : track) {
      ros::Time stamp = measurement.time_point;
      beam::opt<Eigen::Matrix4d> T = this->visual_map_->getPose(stamp);
      // check if the pose is in the graph (keyframe)
      if (T.has_value()) {
        pixels.push_back(measurement.value.cast<int>());
        T_cam_world_v.push_back(T.value());
      }
    }
    if (T_cam_world_v.size() == 2) {
      beam::opt<Eigen::Vector3d> point =
          beam_cv::Triangulation::TriangulatePoint(
              cam_model_, cam_model_, T_cam_world_v[0].inverse(),
              T_cam_world_v[1].inverse(), pixels[0], pixels[1]);
      return point;
    } else {
      return {};
    }
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

}} // namespace beam_models::camera_to_camera
