#include <beam_models/camera_to_camera/visual_inertial_odom.h>
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
PLUGINLIB_EXPORT_CLASS(beam_models::camera_to_camera::VisualInertialOdom,
                       fuse_core::SensorModel)

namespace beam_models { namespace camera_to_camera {

VisualInertialOdom::VisualInertialOdom()
    : fuse_core::AsyncSensorModel(1), device_id_(fuse_core::uuid::NIL) {}

void VisualInertialOdom::onInit() {
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
  this->tracker_ = std::make_shared<beam_cv::Tracker>(detector, descriptor, matcher,
                                                params_.window_size);
  // subscribe to image topic
  image_subscriber_ = private_node_handle_.subscribe(
      params_.image_topic, 1000, &VisualInertialOdom::processImage, this);
  // subscribe to imu topic
  imu_subscriber_ = private_node_handle_.subscribe(
      params_.imu_topic, 10000, &VisualInertialOdom::processIMU, this);
  // make initializer
  Eigen::Matrix4d T_body_cam = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_body_imu = Eigen::Matrix4d::Identity();
  Eigen::Vector4d imu_intrinsics(params_.imu_intrinsics.data());
  initializer_ =
      std::make_shared<beam_models::camera_to_camera::VIOInitializer>(
          cam_model_, T_body_cam, T_body_imu, imu_intrinsics);
}

void VisualInertialOdom::processImage(const sensor_msgs::Image::ConstPtr& msg) {
  // get message info
  image_buffer_.push(*msg);
  sensor_msgs::Image img_msg = image_buffer_.front();
  ros::Time img_time = img_msg.header.stamp;
  /**************************************************************************
   *              Add IMU messages to buffer or initializer                 *
   **************************************************************************/
  while (imu_buffer_.front().header.stamp < img_time && !imu_buffer_.empty()) {
    sensor_msgs::Imu imu_msg = imu_buffer_.front();
    ros::Time imu_time = imu_msg.header.stamp;
    Eigen::Vector3d ang_vel{imu_msg.angular_velocity.x,
                            imu_msg.angular_velocity.y,
                            imu_msg.angular_velocity.z};
    Eigen::Vector3d lin_accel{imu_msg.linear_acceleration.x,
                              imu_msg.linear_acceleration.y,
                              imu_msg.linear_acceleration.z};
    if (!initializer_->Initialized()) {
      initializer_->AddIMU(ang_vel, lin_accel, imu_time);
    } else {
      // preintegrator.PopulateBuffer(msg);
    }
    imu_buffer_.pop();
  }
  /**************************************************************************
   *                    Add Image to map or initializer                     *
   **************************************************************************/
  if (!imu_buffer_.empty()) {
    img_num_++;
    cv::Mat image = this->extractImage(img_msg);
    tracker_->AddImage(image, img_time);
    if (!initializer_->Initialized()) {
      initializer_->AddImage(image, img_time);
      // preintegrator.SetStart(img_time);
    } else {
      std::cout << "Image added at: " << img_time << std::endl;
      // this->RegisterFrame(cur_img, cur_time)
    }
    image_buffer_.pop();
  }
}

void VisualInertialOdom::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_buffer_.push(*msg);
}

void VisualInertialOdom::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  this->graph_ = std::move(graph);
  if (!graph_initialized) graph_initialized = true;
}

void VisualInertialOdom::onStop() {}

std::shared_ptr<fuse_core::Transaction>
    VisualInertialOdom::initMap(ros::Time cur_time) {
  // auto transaction = fuse_core::Transaction::make_shared();
  // // get poses from initialization
  // std::unordered_map<double, Eigen::Matrix4d> poses =
  // initializer_->GetPoses(); Eigen::Matrix4d T = poses[cur_time.toSec()];
  // Eigen::Quaterniond q;
  // Eigen::Vector3d p;
  // beam::TransformMatrixToQuaternionAndTranslation(T, q, p);
  // // create fuse pose variables
  // fuse_variables::Orientation3DStamped::SharedPtr camera_orientation =
  //     fuse_variables::Orientation3DStamped::make_shared(cur_time);
  // camera_orientation->w() = q.w();
  // camera_orientation->x() = q.x();
  // camera_orientation->y() = q.y();
  // camera_orientation->z() = q.z();
  // fuse_variables::Position3DStamped::SharedPtr camera_position =
  //     fuse_variables::Position3DStamped::make_shared(cur_time);
  // camera_position->x() = p[0];
  // camera_position->y() = p[1];
  // camera_position->z() = p[2];
  // // add pose to transaction
  // transaction->stamp(cur_time);
  // transaction->addVariable(camera_orientation);
  // transaction->addVariable(camera_position);
  // // find landmarks in current image
  // std::vector<uint64_t> lm_ids = tracker_->GetLandmarkIDsInImage(cur_time);
  // for (int j = 0; j < lm_ids.size(); j++) {
  //   // get landmark track
  //   std::vector<beam_containers::LandmarkMeasurement<int>> track =
  //       tracker_->GetTrack(lm_ids[j]);
  //   // get pixel of this landmark in the current image
  //   Eigen::Vector2d pixel = tracker_->Get(cur_time, lm_ids[j]);
  //   // add triangulated point to landmark map
  //   if (track.size() >= 2) {
  //     uint64_t id = track[0].landmark_id;

  //     std::vector<Eigen::Matrix4d> T_cam_world_v;
  //     std::vector<Eigen::Vector2i> pixels;
  //     for (auto& measurement : track) {
  //       T_cam_world_v.push_back(poses[measurement.time_point.toSec()]);
  //       pixels.push_back(measurement.value.cast<int>());
  //     }
  //     beam::opt<Eigen::Vector3d> point =
  //         beam_cv::Triangulation::TriangulatePoint(cam_model_, T_cam_world_v,
  //                                                  pixels);

  //     // add triangulated point to landmark map
  //     fuse_variables::Position3D::SharedPtr landmark =
  //         fuse_variables::Position3D::make_shared(std::to_string(id).c_str());
  //     landmark->x() = point.value()[0];
  //     landmark->y() = point.value()[1];
  //     landmark->z() = point.value()[2];
  //     landmark_positions_[id] = landmark;
  //     transaction->addVariable(landmark);
  //     // create and add visual constraint
  //     fuse_constraints::VisualConstraint::SharedPtr vis_constraint =
  //         fuse_constraints::VisualConstraint::make_shared(
  //             source_, *camera_orientation, *camera_position, *landmark,
  //             pixel, cam_model_);
  //     transaction->addConstraint(vis_constraint);
  //   }
  // }
  // return transaction;
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

fuse_variables::Orientation3DStamped::SharedPtr
    VisualInertialOdom::getCameraOrientation(const ros::Time& stamp) {
  std::string orientation_3d_stamped_type =
      "fuse_variables::Orientation3DStamped";

  fuse_variables::Orientation3DStamped::SharedPtr corr_orientation =
      fuse_variables::Orientation3DStamped::make_shared();
  if (graph_initialized) {
    auto corr_orientation_uuid = fuse_core::uuid::generate(
        orientation_3d_stamped_type, stamp, fuse_core::uuid::NIL);
    try {
      *corr_orientation =
          dynamic_cast<const fuse_variables::Orientation3DStamped&>(
              graph_->getVariable(corr_orientation_uuid));
      tracker_orientations_.erase(stamp.toSec());
    } catch (const std::out_of_range& oor) {
      corr_orientation = tracker_orientations_[stamp.toSec()];
    }
  } else {
    corr_orientation = tracker_orientations_[stamp.toSec()];
  }
  return corr_orientation;
}

fuse_variables::Position3DStamped::SharedPtr
    VisualInertialOdom::getCameraPosition(const ros::Time& stamp) {
  std::string position_3d_stamped_type = "fuse_variables::Position3DStamped";

  fuse_variables::Position3DStamped::SharedPtr corr_position =
      fuse_variables::Position3DStamped::make_shared();
  if (graph_initialized) {
    auto corr_position_uuid = fuse_core::uuid::generate(
        position_3d_stamped_type, stamp, fuse_core::uuid::NIL);
    try {
      *corr_position = dynamic_cast<const fuse_variables::Position3DStamped&>(
          graph_->getVariable(corr_position_uuid));
      tracker_positions_.erase(stamp.toSec());
    } catch (const std::out_of_range& oor) {
      corr_position = tracker_positions_[stamp.toSec()];
    }
  } else {
    corr_position = tracker_positions_[stamp.toSec()];
  }
  return corr_position;
}

fuse_variables::Position3D::SharedPtr
    VisualInertialOdom::getLandmark(uint64_t landmark_id) {
  std::string position_3d_type = "fuse_variables::Position3D";
  fuse_variables::Position3D::SharedPtr landmark =
      fuse_variables::Position3D::make_shared();
  if (graph_initialized) {
    auto landmark_uuid = fuse_core::uuid::generate(
        position_3d_type, std::to_string(landmark_id).c_str());
    try {
      *landmark = dynamic_cast<const fuse_variables::Position3D&>(
          graph_->getVariable(landmark_uuid));
      landmark_positions_.erase(landmark_id);
    } catch (const std::out_of_range& oor) {
      landmark = landmark_positions_[landmark_id];
    }
  } else {
    landmark = landmark_positions_[landmark_id];
  }
  return landmark;
}

}} // namespace beam_models::camera_to_camera
