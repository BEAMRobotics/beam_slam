#include <bs_publishers/odometry_3d_publisher.h>

#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <bs_constraints/motion/unicycle_3d_predict.h>

// Register this publisher with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_publishers::Odometry3DPublisher, fuse_core::Publisher)

namespace bs_publishers {

Odometry3DPublisher::Odometry3DPublisher()
    : fuse_core::AsyncPublisher(1),
      device_id_(fuse_core::uuid::NIL),
      latest_stamp_(Synchronizer::TIME_ZERO) {}

void Odometry3DPublisher::onInit() {
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);

  params_.loadFromROS(private_node_handle_);

  if (params_.world_frame_id == params_.map_frame_id) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(params_.tf_cache_time);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
        *tf_buffer_, private_node_handle_);
  }

  odom_pub_ = private_node_handle_.advertise<nav_msgs::Odometry>(
      ros::names::resolve(params_.topic), params_.queue_size);

  tf_publish_timer_ = private_node_handle_.createTimer(
      ros::Duration(1.0 / params_.tf_publish_frequency),
      &Odometry3DPublisher::tfPublishTimerCallback, this, false, false);
}

void Odometry3DPublisher::notifyCallback(
    fuse_core::Transaction::ConstSharedPtr transaction,
    fuse_core::Graph::ConstSharedPtr graph) {
  // Find the most recent common timestamp
  latest_stamp_ = synchronizer_.findLatestCommonStamp(*transaction, *graph);
  if (latest_stamp_ == Synchronizer::TIME_ZERO) {
    ROS_WARN_STREAM_THROTTLE(
        10.0,
        "Failed to find a matching set of position and orientation variables.");
    return;
  }

  // Get the pose values associated with the selected timestamp
  fuse_core::UUID position_uuid;
  fuse_core::UUID orientation_uuid;
  fuse_core::UUID velocity_linear_uuid;
  fuse_core::UUID velocity_angular_uuid;

  if (!getState(*graph, latest_stamp_, device_id_, position_uuid,
                orientation_uuid, velocity_linear_uuid, velocity_angular_uuid,
                odom_output_)) {
    return;
  }

  odom_output_.header.frame_id = params_.world_frame_id;
  odom_output_.header.stamp = latest_stamp_;
  odom_output_.child_frame_id = params_.base_link_output_frame_id;

  // Don't waste CPU computing the covariance if nobody is listening
  if (odom_pub_.getNumSubscribers() > 0) {
    try {
      std::vector<std::pair<fuse_core::UUID, fuse_core::UUID>>
          covariance_requests;
      covariance_requests.emplace_back(position_uuid, position_uuid);
      covariance_requests.emplace_back(position_uuid, orientation_uuid);
      covariance_requests.emplace_back(orientation_uuid, orientation_uuid);
      covariance_requests.emplace_back(velocity_linear_uuid,
                                       velocity_linear_uuid);
      covariance_requests.emplace_back(velocity_linear_uuid,
                                       velocity_angular_uuid);
      covariance_requests.emplace_back(velocity_angular_uuid,
                                       velocity_angular_uuid);

      std::vector<std::vector<double>> covariance_matrices;
      graph->getCovariance(covariance_requests, covariance_matrices,
                           params_.covariance_options);

      odom_output_.pose.covariance[0] = covariance_matrices[0][0];
      odom_output_.pose.covariance[1] = covariance_matrices[0][1];
      odom_output_.pose.covariance[5] = covariance_matrices[1][0];
      odom_output_.pose.covariance[6] = covariance_matrices[0][2];
      odom_output_.pose.covariance[7] = covariance_matrices[0][3];
      odom_output_.pose.covariance[11] = covariance_matrices[1][1];
      odom_output_.pose.covariance[30] = covariance_matrices[1][0];
      odom_output_.pose.covariance[31] = covariance_matrices[1][1];
      odom_output_.pose.covariance[35] = covariance_matrices[2][0];

      odom_output_.twist.covariance[0] = covariance_matrices[3][0];
      odom_output_.twist.covariance[1] = covariance_matrices[3][1];
      odom_output_.twist.covariance[5] = covariance_matrices[4][0];
      odom_output_.twist.covariance[6] = covariance_matrices[3][2];
      odom_output_.twist.covariance[7] = covariance_matrices[3][3];
      odom_output_.twist.covariance[11] = covariance_matrices[4][1];
      odom_output_.twist.covariance[30] = covariance_matrices[4][0];
      odom_output_.twist.covariance[31] = covariance_matrices[4][1];
      odom_output_.twist.covariance[35] = covariance_matrices[5][0];
    } catch (const std::exception& e) {
      ROS_WARN_STREAM(
          "An error occurred computing the covariance information for "
          << latest_stamp_
          << ". "
             "The covariance will be set to zero.\n"
          << e.what());
      std::fill(odom_output_.pose.covariance.begin(),
                odom_output_.pose.covariance.end(), 0.0);
      std::fill(odom_output_.twist.covariance.begin(),
                odom_output_.twist.covariance.end(), 0.0);
    }
    odom_pub_.publish(odom_output_);
  }
}

void Odometry3DPublisher::onStart() {
  synchronizer_ = Synchronizer(device_id_);
  latest_stamp_ = Synchronizer::TIME_ZERO;
  odom_output_ = nav_msgs::Odometry();
  tf_publish_timer_.start();
}

void Odometry3DPublisher::onStop() {
  tf_publish_timer_.stop();
}

bool Odometry3DPublisher::getState(
    const fuse_core::Graph& graph, const ros::Time& stamp,
    const fuse_core::UUID& device_id, fuse_core::UUID& position_uuid,
    fuse_core::UUID& orientation_uuid, fuse_core::UUID& velocity_linear_uuid,
    fuse_core::UUID& velocity_angular_uuid, nav_msgs::Odometry& state) {
  try {
    position_uuid = fuse_variables::Position3DStamped(stamp, device_id).uuid();
    auto position_variable =
        dynamic_cast<const fuse_variables::Position3DStamped&>(
            graph.getVariable(position_uuid));

    orientation_uuid =
        fuse_variables::Orientation3DStamped(stamp, device_id).uuid();
    auto orientation_variable =
        dynamic_cast<const fuse_variables::Orientation3DStamped&>(
            graph.getVariable(orientation_uuid));

    velocity_linear_uuid =
        fuse_variables::VelocityLinear3DStamped(stamp, device_id).uuid();
    auto velocity_linear_variable =
        dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(
            graph.getVariable(velocity_linear_uuid));

    velocity_angular_uuid =
        fuse_variables::VelocityAngular3DStamped(stamp, device_id).uuid();
    auto velocity_angular_variable =
        dynamic_cast<const fuse_variables::VelocityAngular3DStamped&>(
            graph.getVariable(velocity_angular_uuid));

    state.pose.pose.position.x = position_variable.x();
    state.pose.pose.position.y = position_variable.y();
    state.pose.pose.position.z = position_variable.z();
    state.pose.pose.orientation = tf2::toMsg(
        tf2::Quaternion(orientation_variable.x(), orientation_variable.y(),
                        orientation_variable.z(), orientation_variable.w()));

    state.twist.twist.linear.x = velocity_linear_variable.x();
    state.twist.twist.linear.y = velocity_linear_variable.y();
    state.twist.twist.linear.z = velocity_linear_variable.z();
    state.twist.twist.angular.x = velocity_angular_variable.roll();
    state.twist.twist.angular.y = velocity_angular_variable.pitch();
    state.twist.twist.angular.z = velocity_angular_variable.yaw();
  } catch (const std::exception& e) {
    ROS_WARN_STREAM_THROTTLE(10.0, "Failed to find a state at time "
                                       << stamp << ". Error" << e.what());
    return false;
  } catch (...) {
    ROS_WARN_STREAM_THROTTLE(10.0, "Failed to find a state at time "
                                       << stamp << ". Error: unknown");
    return false;
  }

  return true;
}

void Odometry3DPublisher::tfPublishTimerCallback(const ros::TimerEvent& event) {
  if (latest_stamp_ == Synchronizer::TIME_ZERO) {
    ROS_WARN_STREAM_THROTTLE(10.0,
                             "No valid state data yet. Delaying tf broadcast.");
    return;
  }

  tf2::Transform pose;
  tf2::fromMsg(odom_output_.pose.pose, pose);

  geometry_msgs::TransformStamped trans;
  trans.header = odom_output_.header;
  trans.child_frame_id = odom_output_.child_frame_id;

  // If requested, we need to project our state forward in time using the 3D
  // kinematic model
  if (params_.predict_to_current_time) {
    tf2::Vector3 velocity_linear;
    tf2::Vector3 velocity_angular;

    tf2::fromMsg(odom_output_.twist.twist.linear, velocity_linear);
    tf2::fromMsg(odom_output_.twist.twist.angular, velocity_angular);
    tf2::Vector3 unused_acc;
    tf2::Vector3 unused_angular_vel;

    bs_constraints::predict(
        pose, velocity_linear, velocity_angular, unused_acc,
        event.current_real.toSec() - odom_output_.header.stamp.toSec(), pose,
        velocity_linear, unused_angular_vel, unused_acc);

    trans.header.stamp = event.current_real;
  }

  trans.transform.translation.x = pose.getOrigin().x();
  trans.transform.translation.y = pose.getOrigin().y();
  trans.transform.translation.z =
      pose.getOrigin().z(); // odom_output_.pose.pose.position.z;
  trans.transform.rotation = tf2::toMsg(pose.getRotation());

  if (params_.world_frame_id == params_.map_frame_id) {
    try {
      auto base_to_odom = tf_buffer_->lookupTransform(
          params_.base_link_frame_id, params_.odom_frame_id, latest_stamp_,
          params_.tf_timeout);

      geometry_msgs::TransformStamped map_to_odom;
      tf2::doTransform(base_to_odom, map_to_odom, trans);
      map_to_odom.child_frame_id = params_.odom_frame_id;
      trans = map_to_odom;
    } catch (const std::exception& e) {
      ROS_WARN_STREAM_THROTTLE(5.0, "Could not lookup the "
                                        << params_.base_link_frame_id << "->"
                                        << params_.odom_frame_id
                                        << " transform. Error: " << e.what());

      return;
    }
  }

  tf_broadcaster_.sendTransform(trans);
}

} // namespace bs_publishers
