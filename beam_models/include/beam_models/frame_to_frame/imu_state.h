#pragma once

#include <fuse_core/graph.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

#include <beam_common/utils.h>
#include <beam_variables/imu_bias_stamped.h>

namespace beam_models { namespace frame_to_frame {

class ImuState {
public:
  using Ptr = std::shared_ptr<ImuState>;

  ImuState() = default;

  ImuState(const ros::Time& time) : stamp_(time) {
    this->InstantiateFuseVariables();
    this->SetPosition(0, 0, 0);
    this->SetVelocity(0, 0, 0);
    this->SetOrientation(1, 0, 0, 0);
    this->SetBiasAcceleration(0, 0, 0);
    this->SetBiasGyroscope(0, 0, 0);
  }

  ImuState(const ros::Time& time, const double& bias_acceleration_init,
           const double& bias_gyroscope_init)
      : stamp_(time) {
    this->InstantiateFuseVariables();
    this->SetPosition(0, 0, 0);
    this->SetVelocity(0, 0, 0);
    this->SetOrientation(1, 0, 0, 0);
    this->SetBiasAcceleration(bias_acceleration_init, bias_acceleration_init, 
                              bias_acceleration_init);
    this->SetBiasGyroscope(bias_gyroscope_init, bias_gyroscope_init, 
                           bias_gyroscope_init);
  }

  ImuState(const ros::Time& time, const Eigen::Matrix4d& T_WORLD_IMU,
           const fuse_core::Vector3d& velocity,
           const fuse_core::Vector3d& bias_acceleration,
           const fuse_core::Vector3d& bias_gyroscope)
      : stamp_(time) {
    this->InstantiateFuseVariables();
    beam_common::EigenTransformToFusePose(T_WORLD_IMU, position_, orientation_);
    this->SetVelocity(velocity);
    this->SetBiasAcceleration(bias_acceleration);
    this->SetBiasGyroscope(bias_gyroscope);
  }

  ImuState(const ros::Time& time, const fuse_core::Vector3d& position,
           const fuse_core::Vector3d& velocity,
           const Eigen::Quaterniond& orientation,
           const fuse_core::Vector3d& bias_acceleration,
           const fuse_core::Vector3d& bias_gyroscope)
      : stamp_(time) {
    this->InstantiateFuseVariables();
    this->SetPosition(position);
    this->SetVelocity(velocity);
    this->SetOrientation(orientation);
    this->SetBiasAcceleration(bias_acceleration);
    this->SetBiasGyroscope(bias_gyroscope);
  }

  bool Update(const fuse_core::Graph::ConstSharedPtr& graph_msg) {
    if (graph_msg->variableExists(position_.uuid()) &&
        graph_msg->variableExists(velocity_.uuid()) &&
        graph_msg->variableExists(orientation_.uuid()) &&
        graph_msg->variableExists(bias_acceleration_.uuid()) &&
        graph_msg->variableExists(bias_gyroscope_.uuid())) {
      position_ = dynamic_cast<const fuse_variables::Position3DStamped&>(
          graph_msg->getVariable(position_.uuid()));
      velocity_ = dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(
          graph_msg->getVariable(velocity_.uuid()));
      orientation_ = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
          graph_msg->getVariable(orientation_.uuid()));
      bias_acceleration_ = dynamic_cast<const beam_variables::ImuBiasStamped&>(
          graph_msg->getVariable(bias_acceleration_.uuid()));
      bias_gyroscope_ = dynamic_cast<const beam_variables::ImuBiasStamped&>(
          graph_msg->getVariable(bias_gyroscope_.uuid()));
      updates_++;
      return true;
    }
    return false;
  }

  void InstantiateFuseVariables(const ros::Time& time) {
    position_ = fuse_variables::Position3DStamped(time, fuse_core::uuid::NIL);
    velocity_ =
        fuse_variables::VelocityLinear3DStamped(time, fuse_core::uuid::NIL);
    orientation_ =
        fuse_variables::Orientation3DStamped(time, fuse_core::uuid::NIL);
    bias_acceleration_ =
        beam_variables::ImuBiasStamped(time, fuse_core::uuid::NIL);
    bias_gyroscope_ =
        beam_variables::ImuBiasStamped(time, fuse_core::uuid::NIL);
  }

  void InstantiateFuseVariables() {
    position_ = fuse_variables::Position3DStamped(stamp_, fuse_core::uuid::NIL);
    velocity_ =
        fuse_variables::VelocityLinear3DStamped(stamp_, fuse_core::uuid::NIL);
    orientation_ =
        fuse_variables::Orientation3DStamped(stamp_, fuse_core::uuid::NIL);
    bias_acceleration_ =
        beam_variables::ImuBiasStamped(stamp_, fuse_core::uuid::NIL);
    bias_gyroscope_ =
        beam_variables::ImuBiasStamped(stamp_, fuse_core::uuid::NIL);
  }

  inline int Updates() const { return updates_; }

  Eigen::Matrix4d T_WORLD_IMU() const {
    Eigen::Matrix4d T_WORLD_IMU{Eigen::Matrix4d::Identity()};
    beam_common::FusePoseToEigenTransform(position_, orientation_, T_WORLD_IMU);
    return T_WORLD_IMU;
  }

  ros::Time Stamp() const { return stamp_; }

  fuse_variables::Position3DStamped Position() const { return position_; }

  fuse_variables::VelocityLinear3DStamped Velocity() const { return velocity_; }

  fuse_variables::Orientation3DStamped Orientation() const {
    return orientation_;
  }

  beam_variables::ImuBiasStamped BiasAcceleration() const {
    return bias_acceleration_;
  }

  beam_variables::ImuBiasStamped BiasGyroscope() const {
    return bias_gyroscope_;
  }

  void Set_T_WORLD_IMU(const Eigen::Matrix4d& T_WORLD_IMU) {
    beam_common::EigenTransformToFusePose(T_WORLD_IMU, position_, orientation_);
  }

  void SetPosition(const double& x, const double& y, const double& z) {
    position_.x() = x;
    position_.y() = y;
    position_.z() = z;
  }

  void SetPosition(const fuse_core::Vector3d& position) {
    position_.x() = position[0];
    position_.y() = position[1];
    position_.z() = position[2];
  }

  void SetVelocity(const double& x, const double& y, const double& z) {
    velocity_.x() = x;
    velocity_.y() = y;
    velocity_.z() = z;
  }

  void SetVelocity(const fuse_core::Vector3d& velocity) {
    velocity_.x() = velocity[0];
    velocity_.y() = velocity[1];
    velocity_.z() = velocity[2];
  }

  void SetOrientation(const double& w, const double& x, const double& y,
                      const double& z) {
    orientation_.w() = w;
    orientation_.x() = x;
    orientation_.y() = y;
    orientation_.z() = z;
  }

  void SetOrientation(const Eigen::Quaterniond& orientation) {
    orientation_.w() = orientation.w();
    orientation_.x() = orientation.x();
    orientation_.y() = orientation.y();
    orientation_.z() = orientation.z();
  }

  void SetBiasAcceleration(const double& ba_init) {
    bias_acceleration_.x() = ba_init;
    bias_acceleration_.y() = ba_init;
    bias_acceleration_.z() = ba_init;
  }

  void SetBiasAcceleration(const double& x, const double& y, const double& z) {
    bias_acceleration_.x() = x;
    bias_acceleration_.y() = y;
    bias_acceleration_.z() = z;
  }

  void SetBiasAcceleration(const fuse_core::Vector3d& bias_acceleration) {
    bias_acceleration_.x() = bias_acceleration[0];
    bias_acceleration_.y() = bias_acceleration[1];
    bias_acceleration_.z() = bias_acceleration[2];
  }

  void SetBiasGyroscope(const double& bg_init) {
    bias_gyroscope_.x() = bg_init;
    bias_gyroscope_.y() = bg_init;
    bias_gyroscope_.z() = bg_init;
  }

  void SetBiasGyroscope(const double& x, const double& y, const double& z) {
    bias_gyroscope_.x() = x;
    bias_gyroscope_.y() = y;
    bias_gyroscope_.z() = z;
  }

  void SetBiasGyroscope(const fuse_core::Vector3d& bias_gyroscope) {
    bias_gyroscope_.x() = bias_gyroscope[0];
    bias_gyroscope_.y() = bias_gyroscope[1];
    bias_gyroscope_.z() = bias_gyroscope[2];
  }

  void Print(std::ostream& stream = std::cout) const {
    stream << "  Stamp: " << stamp_ << "\n"
           << "  Number of Updates: " << updates_ << "\n"
           << "  Position:\n"
           << "  - x: " << position_.x() << "\n"
           << "  - y: " << position_.y() << "\n"
           << "  - z: " << position_.z() << "\n"
           << "  Velocity:\n"
           << "  - x: " << velocity_.x() << "\n"
           << "  - y: " << velocity_.y() << "\n"
           << "  - z: " << velocity_.z() << "\n"
           << "  Orientation:\n"
           << "  - w: " << orientation_.w() << "\n"
           << "  - x: " << orientation_.x() << "\n"
           << "  - y: " << orientation_.y() << "\n"
           << "  - z: " << orientation_.z() << "\n"
           << "  Bias Acceleration:\n"
           << "  - x: " << bias_acceleration_.x() << "\n"
           << "  - y: " << bias_acceleration_.y() << "\n"
           << "  - z: " << bias_acceleration_.z() << "\n"
           << "  Bias Gyroscope:\n"
           << "  - x: " << bias_gyroscope_.x() << "\n"
           << "  - y: " << bias_gyroscope_.y() << "\n"
           << "  - z: " << bias_gyroscope_.z() << "\n";
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  int updates_{0};
  ros::Time stamp_;
  fuse_variables::Position3DStamped position_;
  fuse_variables::VelocityLinear3DStamped velocity_;
  fuse_variables::Orientation3DStamped orientation_;
  beam_variables::ImuBiasStamped bias_acceleration_;
  beam_variables::ImuBiasStamped bias_gyroscope_;
};

}}  // namespace beam_models::frame_to_frame
