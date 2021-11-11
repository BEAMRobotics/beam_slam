#include <bs_models/inertial_odometry.h>
#include <pluginlib/class_list_macros.h>

#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

#include <nlohmann/json.hpp>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::InertialOdometry, fuse_core::SensorModel)

namespace bs_models {

InertialOdometry::InertialOdometry()
    : fuse_core::AsyncSensorModel(1), device_id_(fuse_core::uuid::NIL),
      throttled_imu_callback_(std::bind(&InertialOdometry::processIMU, this,
                                        std::placeholders::_1)) {}

void InertialOdometry::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  inertial_params_.loadFromROS(private_node_handle_);
  calibration_params_.loadFromROS();

  // read imu params
  nlohmann::json J;
  beam::ReadJson(calibration_params_.imu_intrinsics_path, J);
  imu_params_.cov_gyro_noise =
      Eigen::Matrix3d::Identity() * J["cov_gyro_noise"];
  imu_params_.cov_accel_noise =
      Eigen::Matrix3d::Identity() * J["cov_accel_noise"];
  imu_params_.cov_gyro_bias = Eigen::Matrix3d::Identity() * J["cov_gyro_bias"];
  imu_params_.cov_accel_bias =
      Eigen::Matrix3d::Identity() * J["cov_accel_bias"];
}

void InertialOdometry::onStart() {
  // subscribe to topics
  imu_subscriber_ = node_handle_.subscribe<sensor_msgs::Imu>(
      ros::names::resolve(inertial_params_.input_topic), 10000,
      &ThrottledIMUCallback::callback, &throttled_imu_callback_,
      ros::TransportHints().tcpNoDelay(false));
  path_subscriber_ =
      node_handle_.subscribe(inertial_params_.init_path_topic, 10,
                             &InertialOdometry::processInitPath, this);
  // Advertise publishers
  init_odom_publisher_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odometry", 100);
}

void InertialOdometry::processIMU(const sensor_msgs::Imu::ConstPtr &msg) {
  if (imu_preint_) {
    RegisterImuMessage(*msg);
  } else {
    imu_buffer_.push(*msg);
  }
}

void InertialOdometry::processInitPath(
    const InitializedPathMsg::ConstPtr &msg) {
  // Estimate imu biases and gravity using the initial path
  bs_models::ImuPreintegration::EstimateParameters(
      *msg, imu_buffer_, imu_params_, gravity_, bg_, ba_, scale_);
  imu_preint_ =
      std::make_shared<bs_models::ImuPreintegration>(imu_params_, bg_, ba_);
  // get rotation from the estimated gravity to world gravity
  Eigen::Quaterniond q =
      Eigen::Quaterniond::FromTwoVectors(gravity_, GRAVITY_WORLD);
  int i = 0;
  for (auto &pose : msg->poses) {
    // align the pose to the estimated gravity
    Eigen::Matrix4d T;
    bs_common::PoseMsgToTransformationMatrix(pose, T);
    Eigen::Quaterniond ori;
    Eigen::Vector3d pos;
    beam::TransformMatrixToQuaternionAndTranslation(T, ori, pos);
    ori = q * ori;
    pos = q * pos;
    // get stamp of the pose to add
    ros::Time pose_stamp = pose.header.stamp;
    // create fuse variables for the pose
    fuse_variables::Orientation3DStamped::SharedPtr orientation =
        std::make_shared<fuse_variables::Orientation3DStamped>(pose_stamp);
    orientation->w() = ori.w();
    orientation->x() = ori.x();
    orientation->y() = ori.y();
    orientation->z() = ori.z();
    fuse_variables::Position3DStamped::SharedPtr position =
        std::make_shared<fuse_variables::Position3DStamped>(pose_stamp);
    position->x() = pos[0];
    position->y() = pos[1];
    position->z() = pos[2];
    // add imu messages to preint
    while (imu_buffer_.front().header.stamp < pose_stamp &&
           !imu_buffer_.empty()) {
      imu_preint_->AddToBuffer(imu_buffer_.front());
      imu_buffer_.pop();
    }
    // register constraints
    if (i == 0) {
      // estimate velocity at pose_stamp
      Eigen::Vector3d velocity_vec;
      EstimateVelocityFromPath(msg->poses, pose_stamp, velocity_vec);
      // make fuse variable
      fuse_variables::VelocityLinear3DStamped::SharedPtr velocity =
          std::make_shared<fuse_variables::VelocityLinear3DStamped>(pose_stamp);
      velocity->x() = velocity_vec[0];
      velocity->y() = velocity_vec[1];
      velocity->z() = velocity_vec[2];
      imu_preint_->SetStart(pose_stamp, orientation, position, velocity);
    } else {
      // register constraint with this pose
      fuse_core::Transaction::SharedPtr imu_transaction =
          imu_preint_->RegisterNewImuPreintegratedFactor(pose_stamp,
                                                         orientation, position);
      sendTransaction(imu_transaction);
    }
    previous_state = pose_stamp;
    ++i;
  }
  // add the rest of the messages in the imu buffer to the preint class
  while (!imu_buffer_.empty()) {
    RegisterImuMessage(imu_buffer_.front());
    imu_buffer_.pop();
  }
}

void InertialOdometry::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  if (imu_preint_) {
    // make deep copy to make updating graph easier
    fuse_core::Graph::SharedPtr copy = std::move(graph->clone());
    // Update imu preint info with new graph
    imu_preint_->UpdateGraph(copy);
  }
}

void InertialOdometry::RegisterImuMessage(const sensor_msgs::Imu &msg) {
  imu_preint_->AddToBuffer(msg);
  // get pose and publish odometry
  Eigen::Matrix4d T_WORLD_BASELINK;
  std::shared_ptr<Eigen::Matrix<double, 6, 6>> covariance =
      std::make_shared<Eigen::Matrix<double, 6, 6>>();
  imu_preint_->GetPose(T_WORLD_BASELINK, msg.header.stamp, covariance);
  boost::array<float, 36> covariance_flat;
  covariance_flat.fill(0);
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      covariance_flat[i * 6 + j] = (*covariance)(i,j);
    }
  }

  geometry_msgs::PoseStamped pose;
  bs_common::TransformationMatrixToPoseMsg(T_WORLD_BASELINK, msg.header.stamp,
                                           pose);
  nav_msgs::Odometry odom;
  odom.pose.pose = pose.pose;
  odom.pose.covariance = covariance_flat;
  odom.header.stamp = pose.header.stamp;
  odom.header.frame_id = extrinsics_.GetBaselinkFrameId();
  odom.child_frame_id = extrinsics_.GetWorldFrameId();
  init_odom_publisher_.publish(odom);

  // register inertial constraint
  if ((msg.header.stamp.toSec() - previous_state.toSec()) >=
      inertial_params_.keyframe_rate) {
    fuse_core::Transaction::SharedPtr imu_transaction =
        imu_preint_->RegisterNewImuPreintegratedFactor(msg.header.stamp);
    sendTransaction(imu_transaction);
    previous_state = msg.header.stamp;
  }
}

} // namespace bs_models
