#include <bs_models/inertial_odometry.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Time.h>

#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

#include <beam_utils/pcl_conversions.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::InertialOdometry, fuse_core::SensorModel)

namespace bs_models {

InertialOdometry::InertialOdometry()
    : fuse_core::AsyncSensorModel(1),
      device_id_(fuse_core::uuid::NIL),
      throttled_imu_callback_(std::bind(&InertialOdometry::processIMU, this,
                                        std::placeholders::_1)) {}

void InertialOdometry::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
  inertial_params_.loadFromROS(private_node_handle_);
  calibration_params_.loadFromROS();
  imu_params_.LoadFromJSON(calibration_params_.imu_intrinsics_path);
}

void InertialOdometry::onStart() {
  // subscribe to topics
  imu_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Imu>(
      ros::names::resolve(inertial_params_.input_topic),
      inertial_params_.msg_queue_size, &ThrottledIMUCallback::callback,
      &throttled_imu_callback_, ros::TransportHints().tcpNoDelay(false));
  path_subscriber_ =
      private_node_handle_.subscribe("/local_mapper/slam_init/result", 10,
                                     &InertialOdometry::processInitPath, this);
  // Advertise publishers
  init_odom_publisher_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odometry", 100);

  inertial_pose_stamps_publisher_ =
      private_node_handle_.advertise<std_msgs::Time>("pose_stamps", 1000);
}

void InertialOdometry::onStop() {
  imu_subscriber_.shutdown();
  path_subscriber_.shutdown();
}

void InertialOdometry::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  if (imu_preint_) {
    RegisterImuMessage(*msg);
  } else {
    imu_buffer_.push(*msg);
  }
}

void InertialOdometry::processInitPath(
    const InitializedPathMsg::ConstPtr& msg) {
  ROS_INFO("Initializing IMU biases and gravity from initial path.");

  // Estimate imu biases and gravity using the initial path
  bs_models::ImuPreintegration::EstimateParameters(
      *msg, imu_buffer_, imu_params_, gravity_, bg_, ba_, velocities_, scale_);
  imu_preint_ =
      std::make_shared<bs_models::ImuPreintegration>(imu_params_, bg_, ba_);

  // get rotation from the estimated gravity to world gravity
  Eigen::Quaterniond q =
      Eigen::Quaterniond::FromTwoVectors(gravity_, GRAVITY_WORLD);

  int i = 0;
  for (const auto& pose : msg->poses) {
    // align the pose to the estimated gravity
    Eigen::Matrix4d T_WORLD_BASELINK;
    bs_common::PoseMsgToTransformationMatrix(pose, T_WORLD_BASELINK);
    Eigen::Quaterniond init_orientation;
    Eigen::Vector3d init_position;
    beam::TransformMatrixToQuaternionAndTranslation(
        T_WORLD_BASELINK, init_orientation, init_position);
    init_orientation = q * init_orientation;
    init_position = q * init_position;

    // get stamp of the pose to add
    const ros::Time& pose_stamp = pose.header.stamp;

    // create fuse variables for the pose
    fuse_variables::Orientation3DStamped::SharedPtr fuse_orientation =
        std::make_shared<fuse_variables::Orientation3DStamped>(pose_stamp);
    fuse_orientation->w() = init_orientation.w();
    fuse_orientation->x() = init_orientation.x();
    fuse_orientation->y() = init_orientation.y();
    fuse_orientation->z() = init_orientation.z();
    fuse_variables::Position3DStamped::SharedPtr fuse_position =
        std::make_shared<fuse_variables::Position3DStamped>(pose_stamp);
    fuse_position->x() = init_position[0];
    fuse_position->y() = init_position[1];
    fuse_position->z() = init_position[2];

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
      imu_preint_->SetStart(pose_stamp, fuse_orientation, fuse_position,
                            velocity);
    } else {
      // register constraint with this pose
      fuse_core::Transaction::SharedPtr imu_transaction =
          imu_preint_->RegisterNewImuPreintegratedFactor(
              pose_stamp, fuse_orientation, fuse_position);
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
  ROS_INFO("Finished imu initialization.");
}

void InertialOdometry::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  if (imu_preint_) {
    // make deep copy to make updating graph easier
    fuse_core::Graph::SharedPtr graph_copy = std::move(graph->clone());
    // Update imu preint info with new graph
    imu_preint_->UpdateGraph(graph_copy);
  }
}

void InertialOdometry::RegisterImuMessage(const sensor_msgs::Imu& msg) {
  imu_preint_->AddToBuffer(msg);

  // get pose and publish odometry
  Eigen::Matrix4d T_WORLD_BASELINK;
  std::shared_ptr<Eigen::Matrix<double, 6, 6>> covariance =
      std::make_shared<Eigen::Matrix<double, 6, 6>>();
  if (!imu_preint_->GetPose(T_WORLD_BASELINK, msg.header.stamp, covariance)) {
    ROS_ERROR("Error getting pose, skipping imu measurement");
    return;
  }
  boost::array<float, 36> covariance_flat;
  covariance_flat.fill(0);
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      covariance_flat[i * 6 + j] = (*covariance)(i, j);
    }
  }

  geometry_msgs::PoseStamped pose;
  bs_common::TransformationMatrixToPoseMsg(T_WORLD_BASELINK, msg.header.stamp,
                                           pose);
  nav_msgs::Odometry odom_msg;
  odom_msg.pose.pose = pose.pose;
  odom_msg.pose.covariance = covariance_flat;
  odom_msg.header.stamp = pose.header.stamp;
  odom_msg.header.frame_id = extrinsics_.GetWorldFrameId();
  odom_msg.child_frame_id = extrinsics_.GetBaselinkFrameId();
  init_odom_publisher_.publish(odom_msg);

  // register inertial constraint
  if ((msg.header.stamp.toSec() - previous_state.toSec()) >=
      inertial_params_.keyframe_rate) {
    fuse_core::Transaction::SharedPtr imu_transaction =
        imu_preint_->RegisterNewImuPreintegratedFactor(msg.header.stamp);
    if (!imu_transaction) {
      ROS_ERROR(
          "Error registering inertial transaction, skipping imu measurement");
      return;
    }
    std_msgs::Time time_msg;
    time_msg.data = pose.header.stamp;
    inertial_pose_stamps_publisher_.publish(time_msg);
    sendTransaction(imu_transaction);
    previous_state = msg.header.stamp;
  }
}

} // namespace bs_models
