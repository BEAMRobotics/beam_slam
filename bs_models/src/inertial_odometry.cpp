#include <bs_models/inertial_odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>

#include <beam_utils/filesystem.h>
#include <std_msgs/Empty.h>

#include <bs_common/conversions.h>
#include <bs_common/graph_access.h>
#include <bs_constraints/inertial/relative_imu_state_3d_stamped_constraint.h>
#include <fuse_constraints/relative_constraint.h>
#include <fuse_constraints/relative_pose_3d_stamped_constraint.h>

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
  while (!imu_msgs_.empty() &&
         imu_msgs_.rbegin()->first - imu_msgs_.begin()->first >
             buffer_length_) {
    imu_msgs_.erase(imu_msgs_.begin());
  }

  while (!constraint_buffer_.empty() &&
         constraint_buffer_.rbegin()->first -
                 constraint_buffer_.begin()->first >
             buffer_length_) {
    constraint_buffer_.erase(constraint_buffer_.begin());
  }
}

void ImuBuffer::AddConstraint(const ros::Time& start_time,
                              const ros::Time& end_time,
                              const fuse_core::UUID& constraint_uuid) {
  ImuConstraintData data;
  data.constraint_uuid = constraint_uuid;
  data.start_time = start_time;
  data.end_time = end_time;
  constraint_buffer_.emplace(end_time, data);
}

std::map<ros::Time, sensor_msgs::Imu::ConstPtr>
    ImuBuffer::GetImuData(const ros::Time& start_time,
                          const ros::Time& end_time) const {
  if (imu_msgs_.empty()) { return {}; }
  auto iter_start = imu_msgs_.lower_bound(start_time);
  if (iter_start == imu_msgs_.end()) { return {}; }
  std::map<ros::Time, sensor_msgs::Imu::ConstPtr> data;
  for (auto it = iter_start; it != imu_msgs_.end(); it++) {
    if (it->first > end_time) { break; }
    data.insert(*it);
  }
  return data;
}

std::optional<ImuConstraintData>
    ImuBuffer::ExtractConstraintContainingTime(const ros::Time& time) {
  if (constraint_buffer_.empty()) { return {}; }
  for (auto riter = constraint_buffer_.rbegin();
       riter != constraint_buffer_.rend(); riter++) {
    const ImuConstraintData& constraint_data = riter->second;
    if (constraint_data.start_time <= time &&
        constraint_data.end_time >= time) {
      std::optional<ImuConstraintData> constraint_data_copy = constraint_data;
      constraint_buffer_.erase(riter->first);
      return constraint_data_copy;
    }
  }
  BEAM_WARN("No IMU constraint contains the query time: {} s",
            bs_common::ToString(time));
  return {};
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

  // setup publishers
  odometry_publisher_ =
      private_node_handle_.advertise<nav_msgs::Odometry>("odometry", 100);
  reset_publisher_ =
      private_node_handle_.advertise<std_msgs::Empty>("/local_mapper/reset", 1);

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
  ROS_INFO_STREAM("Starting: " << name());
  // subscribe to imu topic
  imu_subscriber_ = private_node_handle_.subscribe<sensor_msgs::Imu>(
      ros::names::resolve(params_.imu_topic), 1000,
      &ThrottledIMUCallback::callback, &throttled_imu_callback_,
      ros::TransportHints().tcpNoDelay(false));

  trigger_subscriber_ = private_node_handle_.subscribe<std_msgs::Time>(
      ros::names::resolve("/local_mapper/inertial_odometry/trigger"), 10,
      &ThrottledTriggerCallback::callback, &throttled_trigger_callback_,
      ros::TransportHints().tcpNoDelay(false));
}

void InertialOdometry::onStop() {
  ROS_INFO_STREAM("Stopping: " << name());
  shutdown();
}

void InertialOdometry::processIMU(const sensor_msgs::Imu::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE(
      "InertialOdometry received IMU measurements: " << msg->header.stamp);
  std::unique_lock<std::mutex> lk(mutex_);
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
  odom_seq_++;
  prev_stamp_ = msg->header.stamp;
}

void InertialOdometry::processTrigger(const std_msgs::Time::ConstPtr& msg) {
  std::unique_lock<std::mutex> lk(mutex_);
  trigger_buffer_.push_back(msg);

  if (!initialized_) { return; }

  while (!trigger_buffer_.empty()) {
    const auto current_trigger = trigger_buffer_.front();
    const ros::Time& time(current_trigger->data);
    if (!bs_common::GetPosition(*most_recent_graph_msg_, time)) { break; }
    trigger_buffer_.pop_front();

    ros::Time last_constraint_time = imu_buffer_.GetLastConstraintTime();
    if (time == last_constraint_time) {
      return;
    } else if (time > last_constraint_time) {
      auto trans = imu_preint_->RegisterNewImuPreintegratedFactor(time);
      if (!trans) { return; }
      trans->stamp(time);
      sendTransaction(trans);
      auto added_constraints_range = trans->addedConstraints();
      // check that only one was added
      if (std::next(added_constraints_range.begin()) !=
          added_constraints_range.end()) {
        BEAM_ERROR(
            "invalid number of constraints added to graph from IMU data");
        return;
      }
      imu_buffer_.AddConstraint(last_trigger_time_, time,
                                added_constraints_range.begin()->uuid());
    } else {
      // this means we have to remove a constraint and replace it with two
      std::optional<ImuConstraintData> maybeConstraintData =
          imu_buffer_.ExtractConstraintContainingTime(time);
      if (maybeConstraintData) {
        BreakupConstraint(time, maybeConstraintData.value());
      }
    }
    last_trigger_time_ = time;
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

void InertialOdometry::onGraphUpdate(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  std::unique_lock<std::mutex> lk(mutex_);
  most_recent_graph_msg_ = graph_msg;
  if (!initialized_) {
    Initialize(graph_msg);
    return;
  }
  imu_preint_->UpdateGraph(graph_msg);

  const auto cur_imu_state = imu_preint_->GetImuState();
  const auto bg_norm = cur_imu_state.GyroBiasVec().norm();
  const auto ba_norm = cur_imu_state.AccelBiasVec().norm();

  if (bg_norm > 1.0) {
    ROS_ERROR_STREAM(name() << ": IMU Gyro Bias estimate too large: " << bg_norm
                            << ". Resetting system.");
    std_msgs::Empty reset;
    reset_publisher_.publish(reset);
  }
  if (ba_norm > 2.5) {
    ROS_ERROR_STREAM(name() << ": IMU Accel Bias estimate too large: "
                            << ba_norm << ". Resetting system.");
    std_msgs::Empty reset;
    reset_publisher_.publish(reset);
  }
}

void InertialOdometry::Initialize(fuse_core::Graph::ConstSharedPtr graph_msg) {
  ROS_INFO("InertialOdometry received initial graph.");
  std::set<ros::Time> timestamps = bs_common::CurrentTimestamps(*graph_msg);

  // get first state in the graph
  const ros::Time first_stamp = *timestamps.cbegin();
  auto accel_bias = bs_common::GetAccelBias(*graph_msg, first_stamp);
  auto gyro_bias = bs_common::GetGyroscopeBias(*graph_msg, first_stamp);
  auto velocity = bs_common::GetVelocity(*graph_msg, first_stamp);
  auto position = bs_common::GetPosition(*graph_msg, first_stamp);
  auto orientation = bs_common::GetOrientation(*graph_msg, first_stamp);

  // init odom frame
  T_ODOM_IMUprev_ =
      bs_common::FusePoseToEigenTransform(*position, *orientation);

  // create imu preint object
  Eigen::Vector3d ba(accel_bias->x(), accel_bias->y(), accel_bias->z());
  Eigen::Vector3d bg(gyro_bias->x(), gyro_bias->y(), gyro_bias->z());
  imu_preint_ = std::make_shared<bs_models::ImuPreintegration>(
      name(), imu_params_, bg, ba, params_.inertial_information_weight, false);

  // set the start
  imu_preint_->SetStart(first_stamp, orientation, position, velocity);

  prev_stamp_ = first_stamp;
  // integrate imu messages between existing states and publish odometry
  for (auto cur = timestamps.begin(); cur != timestamps.end(); cur++) {
    // get the current stamp
    auto cur_stamp = *cur;

    // get the current state
    auto ab = bs_common::GetAccelBias(*graph_msg, cur_stamp);
    auto gb = bs_common::GetGyroscopeBias(*graph_msg, cur_stamp);
    auto v = bs_common::GetVelocity(*graph_msg, cur_stamp);
    auto p = bs_common::GetPosition(*graph_msg, cur_stamp);
    auto o = bs_common::GetOrientation(*graph_msg, cur_stamp);
    if (!gb || !ab || !o || !p || !v) {
      ROS_ERROR("Potential error from slam initialization.");
      throw std::runtime_error("Potential error from slam initialization.");
    }
    imu_preint_->UpdateState(*p, *o, *v, *gb, *ab);
    prev_stamp_ = p->stamp();

    // get the next stamp
    auto next = std::next(cur);
    if (next == timestamps.end()) { break; }
    auto next_stamp = *next;
    // for each imu message between cur_stamp and next_stamp, integrate
    for (const auto& [imu_stamp, imu_msg] : imu_buffer_.GetImuMsgs()) {
      if (imu_stamp <= cur_stamp) {
        continue;
      } else if (imu_stamp <= next_stamp) {
        imu_preint_->AddToBuffer(*imu_msg);
        // get relative pose and publish
        ComputeRelativeMotion(prev_stamp_, imu_stamp);
        odom_seq_++;
        prev_stamp_ = imu_stamp;
      } else {
        break;
      }
    }
    imu_preint_->Clear();
  }

  const ros::Time last_stamp = *timestamps.crbegin();
  last_trigger_time_ = last_stamp;
  // integrate remaining imu messages
  for (const auto& [imu_stamp, imu_msg] : imu_buffer_.GetImuMsgs()) {
    if (imu_stamp <= last_stamp) { continue; }
    imu_preint_->AddToBuffer(*imu_msg);
    // get relative pose and publish
    ComputeRelativeMotion(prev_stamp_, imu_stamp);
    odom_seq_++;
    prev_stamp_ = imu_stamp;
  }

  // add existing constraints to imu buffer
  for (auto& c : graph_msg->getConstraints()) {
    if (c.type() == "bs_constraints::RelativeImuState3DStampedConstraint") {
      auto imu_constraint = dynamic_cast<
          const bs_constraints::RelativeImuState3DStampedConstraint&>(c);
      imu_buffer_.AddConstraint(imu_constraint.GetTime1(),
                                imu_constraint.GetTime2(), c.uuid());
    }
  }

  initialized_ = true;
}

void InertialOdometry::BreakupConstraint(
    const ros::Time& new_trigger_time,
    const ImuConstraintData& constraint_data) {
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(new_trigger_time);

  auto velocity = bs_common::GetVelocity(*most_recent_graph_msg_,
                                         constraint_data.start_time);
  auto position = bs_common::GetPosition(*most_recent_graph_msg_,
                                         constraint_data.start_time);
  auto orientation = bs_common::GetOrientation(*most_recent_graph_msg_,
                                               constraint_data.start_time);

  imu_preint_->SetStart(constraint_data.start_time, orientation, position,
                        velocity);

  // add first half
  bool first_successful = false;
  std::map<ros::Time, sensor_msgs::Imu::ConstPtr> imu_data1 =
      imu_buffer_.GetImuData(constraint_data.start_time, new_trigger_time);
  if (!imu_data1.empty()) {
    for (const auto& [t, imu_msg] : imu_data1) {
      imu_preint_->AddToBuffer(*imu_msg);
    }
    auto imu_trans1 =
        imu_preint_->RegisterNewImuPreintegratedFactor(new_trigger_time);
    if (!imu_trans1) {
      ROS_WARN_STREAM(
          "cannot add constraint for second half of constraint being "
          "broken up. Constraint start time: "
          << bs_common::ToString(constraint_data.start_time)
          << ", constraint end time: " << bs_common::ToString(new_trigger_time)
          << ".");
      ROS_DEBUG_STREAM(
          "ImuPreintegration buffer: " << imu_preint_->PrintBuffer());
    } else {
      imu_buffer_.AddConstraint(constraint_data.start_time, new_trigger_time,
                                imu_trans1->addedConstraints().begin()->uuid());
      transaction->merge(*imu_trans1);
      first_successful = true;
    }
  }

  // add second half
  bool second_successful = false;
  std::map<ros::Time, sensor_msgs::Imu::ConstPtr> imu_data2 =
      imu_buffer_.GetImuData(new_trigger_time, constraint_data.end_time);
  if (!imu_data1.empty()) {
    for (const auto& [t, imu_msg] : imu_data2) {
      imu_preint_->AddToBuffer(*imu_msg);
    }
    auto imu_trans2 = imu_preint_->RegisterNewImuPreintegratedFactor(
        constraint_data.end_time);
    if (!imu_trans2) {
      ROS_WARN_STREAM(
          "cannot add constraint for second half of constraint being "
          "broken up. Constraint start time: "
          << bs_common::ToString(new_trigger_time) << ", constraint end time: "
          << bs_common::ToString(constraint_data.end_time) << ".");
      ROS_DEBUG_STREAM(
          "ImuPreintegration buffer: " << imu_preint_->PrintBuffer());
    } else {
      imu_buffer_.AddConstraint(new_trigger_time, constraint_data.end_time,
                                imu_trans2->addedConstraints().begin()->uuid());
      transaction->merge(*imu_trans2);
      second_successful = true;
    }
  }

  // add zero motion constraint if need be
  if (std::abs((new_trigger_time - constraint_data.start_time).toSec()) <
      0.005) {
    auto maybe_start_position = bs_common::GetPosition(
        *most_recent_graph_msg_, constraint_data.start_time);
    auto maybe_start_orientation = bs_common::GetOrientation(
        *most_recent_graph_msg_, constraint_data.start_time);
    if (maybe_start_position && maybe_start_orientation) {
      bs_common::ImuState start_state(constraint_data.start_time);
      start_state.SetPosition(*maybe_start_position);
      start_state.SetOrientation(*maybe_start_orientation);
      start_state.SetVelocity(Eigen::Vector3d::Zero());
      start_state.SetGyroBias(Eigen::Vector3d::Zero());
      start_state.SetAccelBias(Eigen::Vector3d::Zero());

      bs_common::ImuState new_state(new_trigger_time);
      new_state.SetPosition(start_state.PositionVec());
      new_state.SetOrientation(start_state.OrientationQuat());
      new_state.SetVelocity(start_state.VelocityVec());
      new_state.SetGyroBias(start_state.GyroBiasVec());
      new_state.SetAccelBias(start_state.AccelBiasVec());

      auto zero_motion_transaction = fuse_core::Transaction::make_shared();
      zero_motion_transaction->stamp(new_trigger_time);
      bs_common::AddZeroMotionFactor(name(), start_state, new_state,
                                     zero_motion_transaction);
      transaction->merge(*zero_motion_transaction);
    } else {
      BEAM_ERROR("Cannot retrieve IMU state at time {}, not able to add zero "
                 "motion constraint.",
                 bs_common::ToString(constraint_data.start_time));
    }
  } else if (std::abs((new_trigger_time - constraint_data.end_time).toSec()) <
             0.005) {
    auto maybe_end_position = bs_common::GetPosition(*most_recent_graph_msg_,
                                                     constraint_data.end_time);
    auto maybe_end_orientation = bs_common::GetOrientation(
        *most_recent_graph_msg_, constraint_data.end_time);
    if (maybe_end_position && maybe_end_orientation) {
      bs_common::ImuState end_state(constraint_data.end_time);
      end_state.SetPosition(*maybe_end_position);
      end_state.SetOrientation(*maybe_end_orientation);
      end_state.SetVelocity(Eigen::Vector3d::Zero());
      end_state.SetGyroBias(Eigen::Vector3d::Zero());
      end_state.SetAccelBias(Eigen::Vector3d::Zero());

      bs_common::ImuState new_state(new_trigger_time);
      new_state.SetPosition(end_state.PositionVec());
      new_state.SetOrientation(end_state.OrientationQuat());
      new_state.SetVelocity(end_state.VelocityVec());
      new_state.SetGyroBias(end_state.GyroBiasVec());
      new_state.SetAccelBias(end_state.AccelBiasVec());

      auto zero_motion_transaction = fuse_core::Transaction::make_shared();
      zero_motion_transaction->stamp(new_trigger_time);
      bs_common::AddZeroMotionFactor(name(), new_state, end_state,
                                     zero_motion_transaction);
      transaction->merge(*zero_motion_transaction);
    } else {
      BEAM_ERROR("Cannot retrieve IMU state at time {}, not able to add zero "
                 "motion constraint.",
                 bs_common::ToString(constraint_data.end_time));
    }
  }

  // only remove original if both are successful
  if (second_successful && first_successful) {
    transaction->removeConstraint(constraint_data.constraint_uuid);
  }
  sendTransaction(transaction);
}

void InertialOdometry::shutdown() {
  imu_subscriber_.shutdown();
  trigger_subscriber_.shutdown();
  odom_seq_ = 0;
  initialized_ = false;
  prev_stamp_ = ros::Time(0.0);
  T_ODOM_IMUprev_ = Eigen::Matrix4d::Identity();
  trigger_buffer_.clear();
  imu_buffer_ = ImuBuffer();
  imu_preint_->Reset();
}

} // namespace bs_models