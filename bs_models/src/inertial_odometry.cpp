#include <bs_models/inertial_odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>

#include <bs_common/conversions.h>
#include <bs_common/graph_access.h>
#include <bs_constraints/inertial/relative_imu_state_3d_stamped_constraint.h>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::InertialOdometry, fuse_core::SensorModel);

namespace bs_models {

ImuBuffer::ImuBuffer(double buffer_length_s) {
  buffer_length_ = ros::Duration(buffer_length_s);
}

void ImuBuffer::AddData(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_msgs_.emplace(msg->header.stamp, msg);
  CleanOverflow();
}

void ImuBuffer::CleanOverflow() {
  if (!imu_msgs_.empty()) {
    while (imu_msgs_.rbegin()->first - imu_msgs_.begin()->first >
           buffer_length_) {
      imu_msgs_.erase(imu_msgs_.begin());
    }
  }

  if (!constraint_buffer_.empty()) {
    while (constraint_buffer_.rbegin()->first -
               constraint_buffer_.begin()->first >
           buffer_length_) {
      constraint_buffer_.erase(constraint_buffer_.begin());
    }
  }
}

void ImuBuffer::SetConstraint(const ros::Time& time,
                              const fuse_core::UUID& constraint_uuid) {
  ImuConstraintData data;
  data.constraint_uuid = constraint_uuid;
  constraint_buffer_.emplace(time, data);
  auto it = constraint_buffer_.find(time);
  for (const auto& [_, msg] : imu_msgs_) { it->second.imu_data.push_back(msg); }
  imu_msgs_.clear();
}

void ImuBuffer::AddConstraintData(const ros::Time& time,
                                  const ImuConstraintData& constraint_data) {
  constraint_buffer_.emplace(time, constraint_data);
}

ImuConstraintData
    ImuBuffer::ExtractConstraintContainingTime(const ros::Time& time) {
  for (auto riter = constraint_buffer_.rbegin();
       riter != constraint_buffer_.rend(); riter++) {
    const auto& first_msg_in_constraint = *(riter->second.imu_data.begin());
    BEAM_ERROR("first_msg_in_constraint->header.stamp: {}",
               bs_common::ToString(first_msg_in_constraint->header.stamp));
    if (first_msg_in_constraint->header.stamp <= time) {
      ImuConstraintData extracted_data = riter->second;
      constraint_buffer_.erase(riter->first);
      return extracted_data;
    }
  }
  BEAM_ERROR("no constraint contains the query time: {} s",
             bs_common::ToString(time));
  throw std::runtime_error{"cannot extract constraint"};
}

ros::Time ImuBuffer::GetLastConstraintTime() const {
  if (constraint_buffer_.empty()) { return ros::Time(0); }
  return constraint_buffer_.rbegin()->first;
}

void ImuBuffer::ClearImuMsgs() {
  imu_msgs_.clear();
}

const std::map<ros::Time, sensor_msgs::Imu::ConstPtr>&
    ImuBuffer::GetImuMsgs() const {
  return imu_msgs_;
}

InertialOdometry::InertialOdometry()
    : fuse_core::AsyncSensorModel(2),
      throttled_imu_callback_(std::bind(&InertialOdometry::processIMU, this,
                                        std::placeholders::_1)),
      throttled_trigger_callback_(std::bind(&InertialOdometry::processTrigger,
                                            this, std::placeholders::_1)) {}

void InertialOdometry::onInit() {
  // Read settings from the parameter sever
  calibration_params_.loadFromROS();
  params_.loadFromROS(private_node_handle_);

  // read imu parameters
  nlohmann::json J;
  beam::ReadJson(calibration_params_.imu_intrinsics_path, J);
  imu_params_.cov_prior_noise = J["cov_prior_noise"];
  imu_params_.cov_gyro_noise =
      Eigen::Matrix3d::Identity() * J["cov_gyro_noise"];
  imu_params_.cov_accel_noise =
      Eigen::Matrix3d::Identity() * J["cov_accel_noise"];
  imu_params_.cov_gyro_bias = Eigen::Matrix3d::Identity() * J["cov_gyro_bias"];
  imu_params_.cov_accel_bias =
      Eigen::Matrix3d::Identity() * J["cov_accel_bias"];
}

void InertialOdometry::onStart() {
  // subscribe to imu topic
  imu_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Imu>(
      ros::names::resolve(params_.imu_topic), 1000,
      &ThrottledIMUCallback::callback, &throttled_imu_callback_,
      ros::TransportHints().tcpNoDelay(false));

  trigger_subscriber_ = private_node_handle_.subscribe<std_msgs::Time>(
      ros::names::resolve("/local_mapper/inertial_odometry/trigger"), 10,
      &ThrottledTriggerCallback::callback, &throttled_trigger_callback_,
      ros::TransportHints().tcpNoDelay(false));

  // setup publishers
  odometry_publisher_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odometry", 100);
  pose_publisher_ =
      private_node_handle_.advertise<geometry_msgs::PoseStamped>("pose", 100);
}

void InertialOdometry::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE(
      "InertialOdometry received IMU measurements: " << msg->header.stamp);
  imu_buffer_.AddData(msg);

  // return if its not initialized_
  if (!initialized_) {
    ROS_INFO_THROTTLE(
        1, "inertial odometry not yet initialized, waiting on first graph "
           "update before beginning");
    return;
  }

  // process each imu message as it comes in
  imu_preint_->AddToBuffer(*msg);

  // get relative pose and publish
  ComputeRelativeMotion(prev_stamp_, msg->header.stamp);

  // get world pose and publish
  ComputeAbsolutePose(msg->header.stamp);

  odom_seq_++;
  prev_stamp_ = msg->header.stamp;
}

void InertialOdometry::processTrigger(const std_msgs::Time::ConstPtr& msg) {
  // return if its not initialized_
  if (!initialized_) { return; }

  BEAM_ERROR("processing trig");
  // std::cout << msg->data << "\n";
  const ros::Time& time(msg->data);
  // std::cout << time.toSec() << "\n";
  // std::cout << "TEST0\n";
  ros::Time last_constraint_time = imu_buffer_.GetLastConstraintTime();
  // std::cout << "TEST1\n";
  if (time == last_constraint_time) {
    BEAM_ERROR("trigger time is equal to last constraint time");
    return;
  } else if (time > last_constraint_time) {
    BEAM_ERROR("trigger time ({}) is greater than last constraint time ({})",
               bs_common::ToString(time),
               bs_common::ToString(last_constraint_time));
    auto trans = imu_preint_->RegisterNewImuPreintegratedFactor(time);
    if (!trans) { return; }
    sendTransaction(trans);
    auto added_constraints_range = trans->addedConstraints();
    // check that only one was added
    if (std::next(added_constraints_range.begin()) !=
        added_constraints_range.end()) {
      BEAM_ERROR("invalid number of constraints added to graph from IMU data");
      return;
    }
    imu_buffer_.SetConstraint(time, added_constraints_range.begin()->uuid());
  } else {
    BEAM_ERROR("trigger time ({}) is less than last constraint time ({})",
               bs_common::ToString(time),
               bs_common::ToString(last_constraint_time));
    // this means we have to remove a constraint and replace it with two
    BreakupConstraint(time, imu_buffer_.ExtractConstraintContainingTime(time));
  }
}

void InertialOdometry::ComputeRelativeMotion(const ros::Time& prev_stamp,
                                             const ros ::Time& curr_stamp) {
  Eigen::Vector3d velocity_curr;
  const auto [T_IMUprev_IMUcurr, cov_rel] =
      imu_preint_->GetRelativeMotion(prev_stamp, curr_stamp, velocity_curr);

  // reorder covariance from: roll,pitch,yaw,x,y,z to x,y,z,roll,pitch,yaw
  Eigen::Matrix<double, 6, 6> cov_reordered =
      Eigen::Matrix<double, 6, 6>::Zero();
  cov_reordered.block<3, 3>(0, 0) = cov_rel.block<3, 3>(3, 3);
  cov_reordered.block<3, 3>(0, 3) = cov_rel.block<3, 3>(3, 0);
  cov_reordered.block<3, 3>(3, 0) = cov_rel.block<3, 3>(0, 3);
  cov_reordered.block<3, 3>(3, 3) = cov_rel.block<3, 3>(0, 0);

  // publish relative odometry
  T_ODOM_IMUprev_ = T_ODOM_IMUprev_ * T_IMUprev_IMUcurr;
  auto odom_msg_rel = bs_common::TransformToOdometryMessage(
      curr_stamp, odom_seq_, extrinsics_.GetWorldFrameId(),
      extrinsics_.GetImuFrameId(), T_ODOM_IMUprev_, cov_reordered);
  odometry_publisher_.publish(odom_msg_rel);
}

void InertialOdometry::ComputeAbsolutePose(const ros::Time& curr_stamp) {
  // get world pose and publish
  const auto [T_WORLD_IMU, cov_abs] = imu_preint_->GetPose(curr_stamp);
  geometry_msgs::PoseStamped msg;
  bs_common::EigenTransformToPoseStamped(T_WORLD_IMU, curr_stamp, odom_seq_,
                                         extrinsics_.GetImuFrameId(), msg);
  pose_publisher_.publish(msg);
}

void InertialOdometry::onGraphUpdate(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  if (!initialized_) {
    Initialize(graph_msg);
    return;
  }
  most_recent_graph_msg_ = graph_msg;
  imu_preint_->UpdateGraph(graph_msg);
}

void InertialOdometry::Initialize(fuse_core::Graph::ConstSharedPtr graph_msg) {
  ROS_INFO("InertialOdometry received initial graph.");
  std::set<ros::Time> timestamps = bs_common::CurrentTimestamps(graph_msg);

  // publish poses in initial graph as odometry
  for (const auto& stamp : timestamps) {
    T_ODOM_IMUprev_ = bs_common::FusePoseToEigenTransform(
        *bs_common::GetPosition(graph_msg, stamp),
        *bs_common::GetOrientation(graph_msg, stamp));
    auto odom_msg = bs_common::TransformToOdometryMessage(
        stamp, odom_seq_, extrinsics_.GetWorldFrameId(),
        extrinsics_.GetImuFrameId(), T_ODOM_IMUprev_,
        Eigen::Matrix<double, 6, 6>::Identity() * 1e-5);
    odometry_publisher_.publish(odom_msg);
  }

  // get most recent variables in graph to initialize imu preintegrator
  const ros::Time most_recent_stamp = *timestamps.crbegin();
  auto accel_bias = bs_common::GetAccelBias(graph_msg, most_recent_stamp);
  auto gyro_bias = bs_common::GetGryoscopeBias(graph_msg, most_recent_stamp);
  auto velocity = bs_common::GetVelocity(graph_msg, most_recent_stamp);
  auto position = bs_common::GetPosition(graph_msg, most_recent_stamp);
  auto orientation = bs_common::GetOrientation(graph_msg, most_recent_stamp);

  // create imu preint object
  Eigen::Vector3d ba(accel_bias->x(), accel_bias->y(), accel_bias->z());
  Eigen::Vector3d bg(gyro_bias->x(), gyro_bias->y(), gyro_bias->z());
  imu_preint_ = std::make_shared<bs_models::ImuPreintegration>(
      imu_params_, bg, ba, params_.inertial_info_weight, false);

  // set the start
  imu_preint_->SetStart(most_recent_stamp, orientation, position, velocity);

  // iterate through existing buffer, estimate poses and output
  prev_stamp_ = most_recent_stamp;
  for (const auto& [curr_stamp, imu_msg] : imu_buffer_.GetImuMsgs()) {
    if (curr_stamp < most_recent_stamp) { continue; }
    imu_preint_->AddToBuffer(*imu_msg);

    // get relative pose and publish
    ComputeRelativeMotion(prev_stamp_, curr_stamp);

    // get world pose and publish
    ComputeAbsolutePose(curr_stamp);

    odom_seq_++;
    prev_stamp_ = curr_stamp;
  }
  imu_buffer_.ClearImuMsgs();

  initialized_ = true;
}

void InertialOdometry::BreakupConstraint(
    const ros::Time& new_trigger_time,
    const ImuConstraintData& constraint_data) {
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(ros::Time::now());
  transaction->removeConstraint(constraint_data.constraint_uuid);

  ros::Time start_time = (*constraint_data.imu_data.begin())->header.stamp;
  ros::Time end_time = (*constraint_data.imu_data.rbegin())->header.stamp;

  BEAM_ERROR("start_time: {}", bs_common::ToString(start_time));
  BEAM_ERROR("end_time: {}", bs_common::ToString(end_time));
  BEAM_ERROR("new_trigger_time: {}", bs_common::ToString(new_trigger_time));
  auto velocity = bs_common::GetVelocity(most_recent_graph_msg_, start_time);
  auto position = bs_common::GetPosition(most_recent_graph_msg_, start_time);
  auto orientation =
      bs_common::GetOrientation(most_recent_graph_msg_, start_time);

  imu_preint_->SetStart(start_time, orientation, position, velocity);

  // add first half
  auto imu_msg_iter = constraint_data.imu_data.begin();
  ImuConstraintData constraint_data_first_half;
  while ((*imu_msg_iter)->header.stamp <= new_trigger_time) {
    imu_preint_->AddToBuffer(*(*imu_msg_iter));
    constraint_data_first_half.imu_data.push_back(*imu_msg_iter);
    imu_msg_iter++;
  }
  auto imu_trans1 =
      imu_preint_->RegisterNewImuPreintegratedFactor(new_trigger_time);
  if (!imu_trans1) {
    BEAM_ERROR("cannot add constraint for first half of constraint being "
               "broken up. ImuPreintegration buffer: ");
    std::cout << imu_preint_->PrintBuffer() << "\n";
    throw std::runtime_error{"unable to add IMU preintegration factor"};
  }
  constraint_data_first_half.constraint_uuid =
      imu_trans1->addedConstraints().begin()->uuid();
  imu_buffer_.AddConstraintData(new_trigger_time, constraint_data_first_half);
  transaction->merge(*imu_trans1);

  // add second half
  ImuConstraintData constraint_data_second_half;
  while ((*imu_msg_iter)->header.stamp <= end_time) {
    imu_preint_->AddToBuffer(*(*imu_msg_iter));
    constraint_data_second_half.imu_data.push_back(*imu_msg_iter);
    imu_msg_iter++;
  }
  auto imu_trans2 = imu_preint_->RegisterNewImuPreintegratedFactor(end_time);
  if (!imu_trans2) {
    BEAM_ERROR("cannot add constraint for second half of constraint being "
               "broken up. ImuPreintegration buffer: ");
    std::cout << imu_preint_->PrintBuffer() << "\n";
    throw std::runtime_error{"unable to add IMU preintegration factor"};
  }
  constraint_data_second_half.constraint_uuid =
      imu_trans2->addedConstraints().begin()->uuid();
  imu_buffer_.AddConstraintData(end_time, constraint_data_second_half);
  transaction->merge(*imu_trans2);

  sendTransaction(transaction);
}

} // namespace bs_models