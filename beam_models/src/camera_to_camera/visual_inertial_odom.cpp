#include <beam_models/camera_to_camera/visual_inertial_odom.h>
// fuse
#include <fuse_core/transaction.h>
#include <pluginlib/class_list_macros.h>
// other
#include <cv_bridge/cv_bridge.h>
// libbeam
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_cv/matchers/Matchers.h>

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
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  params_.loadFromROS(private_node_handle_);
  /***********************************************************
   *        Load camera model and Create Map object          *
   ***********************************************************/
  this->cam_model_ =
      beam_calibration::CameraModel::Create(params_.cam_intrinsics_path);
  this->visual_map_ =
      std::make_shared<VisualMap>(this->source_, this->cam_model_, T_body_cam);
  /***********************************************************
   *              Initialize tracker variables               *
   ***********************************************************/
  std::shared_ptr<beam_cv::Matcher> matcher =
      std::make_shared<beam_cv::BFMatcher>(cv::NORM_HAMMING);
  std::shared_ptr<beam_cv::Descriptor> descriptor =
      std::make_shared<beam_cv::ORBDescriptor>();
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::ORBDetector>();
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
  initializer_ = std::make_shared<VIOInitializer>(
      cam_model_, T_imu_cam, params_.imu_intrinsics.data());
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
      if (initializer_->AddImage(image, img_time)) {
        // this transaction adds the first pose, adds the initial map points
        // and initializes the preintegrator object
        auto init_transaction = this->initMap();
        sendTransaction(init_transaction);
        tracker_->AddImage(image, img_time);
        // register the current frame against the map
        // auto frame_transaction = this->registerFrame(img_time);
        // sendTransaction(init_transaction);
      } else {
        tracker_->AddImage(image, img_time);
        std::queue<sensor_msgs::Imu>().swap(temp_imu_buffer_);
      }
    } else {
      /* Keyframe check:
       *  1. Get pose estimate from frame initializer (graph + imu preint)
       *  2. Determine if enough translational movement has occured, OR time
       *     has passed since the last keyframe
       *  3. Check blurriness score of image, and make sure not to add blurry
       * images
       *  https://www.pyimagesearch.com/2015/09/07/blur-detection-with-opencv/
       *  4. Match image to prev keyframe, if matches are below threshold, then
       * register in tracker
       *  5. Otherwise, do pose refinement and store pose
       *
       */

      // tracker_->AddImage(image, img_time);
      // register the current frame against the map
      // auto transaction = this->registerFrame(img_time);
      // sendTransaction(transaction);
    }
    image_buffer_.pop();
  }
}

void VisualInertialOdom::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  // indiscriminantly push imu messages onto its buffer
  imu_buffer_.push(*msg);
}

void VisualInertialOdom::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  /*
   * 1. update the frame initializers graph with this one
   * 2. check the time that is now out of the optimization window (cur_kf_time -
   * lag duration)
   * 3. go through keyframe queue and publish its pose if its older than this
   * time, then remove it
   */
}

void VisualInertialOdom::onStop() {}

std::shared_ptr<fuse_core::Transaction> VisualInertialOdom::initMap() {
  auto transaction = fuse_core::Transaction::make_shared();
  /******************************************************
   *           Add initializer poses to map             *
   ******************************************************/
  std::map<unsigned long, Eigen::Matrix4d> poses = initializer_->GetPoses();
  for (auto& pose : poses) {
    cur_kf_time_.fromNSec(pose.first);
    // this->frame_initializer_->AddCameraPose(pose.second, cur_kf_time_,
    // transaction);
    this->visual_map_->addPose(pose.second, cur_kf_time_, transaction);
  }
  /******************************************************
   *           Add landmarks and constraints            *
   ******************************************************/
  int num_lm = 0;
  std::vector<uint64_t> lm_ids = tracker_->GetLandmarkIDsInImage(cur_kf_time_);
  for (int i = 0; i < lm_ids.size(); i++) {
    uint64_t id = lm_ids[i];
    // get landmark track
    beam_cv::FeatureTrack track = tracker_->GetTrack(id);
    // get pixel of this landmark in the current image
    Eigen::Vector2d pixel_measurement = tracker_->Get(cur_kf_time_, id);
    // triangulate the track and add to transaction
    beam::opt<Eigen::Vector3d> point = this->triangulate(track);
    if (point.has_value()) {
      num_lm++;
      // this->AddLandmark(point.value(), id, transaction);
      // this->AddConstraint(cur_kf_time_, id, transaction);
      this->visual_map_->addLandmark(point.value(), id, transaction);
      this->visual_map_->addConstraint(cur_kf_time_, id, pixel_measurement,
                                       transaction);
    }
  }

  ROS_INFO("Initialized Map Points: %d", num_lm);
  /******************************************************
   *           Initialize the IMU Preintegrator         *
   ******************************************************/
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
  while (temp_imu_buffer_.size() > 0) {
    imu_preint_->PopulateBuffer(temp_imu_buffer_.front());
    temp_imu_buffer_.pop();
  }
  imu_preint_->SetStart(cur_kf_time_,
                        this->visual_map_->getOrientation(cur_kf_time_),
                        this->visual_map_->getPosition(cur_kf_time_), velocity);
  // this->frame_initializer_->SetIMUPreintegator(imu_preint_);
  return transaction;
}

std::shared_ptr<fuse_core::Transaction>
    VisualInertialOdom::registerFrame(const ros::Time& img_time) {
  auto transaction = fuse_core::Transaction::make_shared();
  std::vector<uint64_t> common_landmarks;
  // beam::opt<Eigen::Matrix4d> T_estimate =
      //     this->frame_initializer_->GetCameraPose(img_time);
  // check if image is a keyframe
  if (this->isKeyframe(img_time, common_landmarks, imu_pose_est)) {
    ROS_INFO("New Keyframe Added.");
    // get keypoints
    std::vector<Eigen::Vector2i> pixels;
    std::vector<Eigen::Vector3d> points;
    for (auto& id : common_landmarks) {
      // beam::opt<Eigen::Vector3d> lm = this->GetLandmark(id);
      beam::opt<Eigen::Vector3d> lm = this->visual_map_->getLandmark(id);
      if (lm.has_value()) {
        points.push_back(lm.value());
        pixels.push_back(tracker_->Get(img_time, id).cast<int>());
      }
    }

    ROS_INFO("Available Map Points: %d", points.size());
    // get pose of keyframe
    if (points.size() > 20) {
      // add pose from pnp
      Eigen::Matrix4d T = beam_cv::AbsolutePoseEstimator::RANSACEstimator(
          this->cam_model_, pixels, points);
      this->visual_map_->addPose(T, img_time, transaction);
      // this->frame_initializer_->AddCameraPose(T, img_time, transaction);
    } else {
      // add pose from imu
      this->visual_map_->addPose(T_estimate, img_time, transaction);
      // this->frame_initializer_->AddCameraPose(T, img_time, transaction);
    }
    int new_points = 0;

    // add landmarks/visual constraints
    for (auto& id : common_landmarks) {
      beam::opt<Eigen::Vector3d> lm = this->visual_map_->getLandmark(id);
      // if the landmark hasnt been triangulated then triangulate it
      if (!lm.has_value()) {
        beam_cv::FeatureTrack track = tracker_->GetTrack(id);
        beam::opt<Eigen::Vector3d> point = this->triangulate(track);
        if (point.has_value()) {
          new_points++;
          this->visual_map_->addLandmark(point.value(), id, transaction);
          this->visual_map_->addConstraint(
              img_time, id, tracker_->Get(img_time, id), transaction);
          this->visual_map_->addConstraint(
              cur_kf_time_, id, tracker_->Get(cur_kf_time_, id), transaction);

          // this->AddLandmark(point.value(), id, transaction);
          // this->AddConstraint(cur_kf_time_, id, transaction);
          // this->AddConstraint(img_time, id, transaction);
        }
      } else {
         // this->AddConstraint(img_time, id, transaction);
        this->visual_map_->addConstraint(
            img_time, id, tracker_->Get(img_time, id), transaction);
      }
    }
    ROS_INFO("New Map Points: %d", new_points);
    cur_kf_time_ = img_time;
  }
  return transaction;
}

beam::opt<Eigen::Vector3d>
    VisualInertialOdom::triangulate(beam_cv::FeatureTrack track) {
  if (track.size() >= 2) {
    std::vector<Eigen::Matrix4d> T_cam_world_v;
    std::vector<Eigen::Vector2i> pixels;
    for (auto& measurement : track) {
      // beam::opt<Eigen::Matrix4d> T =
      //     this->frame_initializer_->GetCameraPose(measurement.time_point);
      beam::opt<Eigen::Matrix4d> T =
          this->visual_map_->getPose(measurement.time_point);
      if (T.has_value()) {
        pixels.push_back(measurement.value.cast<int>());
        T_cam_world_v.push_back(T.value());
      } else {
        return {};
      }
    }
    beam::opt<Eigen::Vector3d> point = beam_cv::Triangulation::TriangulatePoint(
        this->cam_model_, T_cam_world_v, pixels);
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

}} // namespace beam_models::camera_to_camera
