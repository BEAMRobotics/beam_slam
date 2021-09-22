#include <bs_models/gt_initializer.h>

#include <pluginlib/class_list_macros.h>

#include <beam_utils/math.h>

#include <bs_common/bs_msgs.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::GTInitializer,
                       fuse_core::SensorModel)

namespace bs_models { 

GTInitializer::GTInitializer() : fuse_core::AsyncSensorModel(1) {}

void GTInitializer::onInit() {
  // load parameters from ros
  calibration_params_.loadFromROS();
  gt_initializer_params_.loadFromROS(private_node_handle_);

  // advertise init path publisher
  results_publisher_ =
      private_node_handle_.advertise<bs_common::InitializedPathMsg>(
          gt_initializer_params_.output_topic, 100);

  // subscribe to imu topic
  imu_subscriber_ = private_node_handle_.subscribe(
      gt_initializer_params_.imu_topic, 100, &GTInitializer::processIMU, this);

  // make pose file frame initializer
  frame_initializer_ =
      std::make_unique<frame_initializers::PoseFileFrameInitializer>(
          gt_initializer_params_.pose_file_path);
  // compute the max # of poses to keep given they are added at ~10hz
  max_poses_ = gt_initializer_params_.trajectory_time_window.toSec() / 0.1;
}

void GTInitializer::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  if (initialization_complete_) return;
  Eigen::Matrix4d T_WORLD_SENSOR;
  bool success = frame_initializer_->GetEstimatedPose(
      T_WORLD_SENSOR, msg->header.stamp, extrinsics_.GetBaselinkFrameId());
  // push poses to trajectory at ~10hz
  if (success && (msg->header.stamp - current_pose_time_).toSec() >= 0.1) {
    current_pose_time_ = msg->header.stamp;
    trajectory_.push_back(T_WORLD_SENSOR);
    times_.push_back(msg->header.stamp);
    if (beam::PassedMotionThreshold(
            trajectory_[0], T_WORLD_SENSOR, 0.0,
            gt_initializer_params_.min_trajectory_length, true, true, false)) {
      ROS_INFO("GT Initializer trajectory long enough. Complete.");
      PublishResults();
      initialization_complete_ = true;
      stop();
    } else if (trajectory_.size() > max_poses_) {
      trajectory_.erase(trajectory_.begin());
      times_.erase(times_.begin());
    }
  }
}

void GTInitializer::PublishResults() {
  bs_common::InitializedPathMsg msg;

  for (uint32_t i = 0; i < trajectory_.size(); i++) {
    Eigen::Matrix4d T = trajectory_[i];

    std_msgs::Header header;
    header.frame_id = extrinsics_.GetBaselinkFrameId();
    header.seq = i;
    header.stamp = times_[i];

    geometry_msgs::Point position;
    position.x = T(0, 3);
    position.y = T(1, 3);
    position.z = T(2, 3);

    Eigen::Matrix3d R = T.block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);

    geometry_msgs::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();

    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose.position = position;
    pose.pose.orientation = orientation;
    msg.poses.push_back(pose);
  }
  results_publisher_.publish(msg);
}

} // namespace bs_models::frame_to_frame
