#pragma once

#include <fuse_core/graph.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

#include <beam_variables/imu_bias_gyro_3d_stamped.h>
#include <beam_variables/imu_bias_accel_3d_stamped.h>

namespace beam_models { namespace frame_to_frame {

class ImuState {
public:
  ImuState() = default;

  ImuState(const ros::Time& time) : stamp_(time) {
    InstantiateFuseVariables();
    SetOrientation(1, 0, 0, 0);
    SetPosition(0, 0, 0);  
    SetVelocity(0, 0, 0); 
    SetBiasGyroscope(0, 0, 0);      
    SetBiasAcceleration(0, 0, 0);
  }

  ImuState(const ros::Time& time, const Eigen::Quaterniond& orientation, 
           const fuse_core::Vector3d& position,
           const fuse_core::Vector3d& velocity)
      : stamp_(time) {
    InstantiateFuseVariables();
    SetOrientation(orientation);
    SetPosition(position);
    SetVelocity(velocity);
    SetBiasGyroscope(0, 0, 0);      
    SetBiasAcceleration(0, 0, 0);
  }

  ImuState(const ros::Time& time, const Eigen::Quaterniond& orientation, 
           const fuse_core::Vector3d& position,
           const fuse_core::Vector3d& velocity,
           const fuse_core::Vector3d& bias_gyroscope,
           const fuse_core::Vector3d& bias_acceleration)
      : stamp_(time) {
    InstantiateFuseVariables();
    SetOrientation(orientation);
    SetPosition(position);
    SetVelocity(velocity);
    SetBiasGyroscope(bias_gyroscope);
    SetBiasAcceleration(bias_acceleration);
  }

  bool Update(const fuse_core::Graph::ConstSharedPtr& graph_msg) {
    if (graph_msg->variableExists(orientation_.uuid()) &&
        graph_msg->variableExists(position_.uuid()) &&
        graph_msg->variableExists(velocity_.uuid()) &&
        graph_msg->variableExists(bias_gyroscope_.uuid()) &&
        graph_msg->variableExists(bias_acceleration_.uuid())) {
      orientation_ = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
          graph_msg->getVariable(orientation_.uuid()));
      position_ = dynamic_cast<const fuse_variables::Position3DStamped&>(
          graph_msg->getVariable(position_.uuid()));       
      velocity_ = dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(
          graph_msg->getVariable(velocity_.uuid()));     
      bias_gyroscope_ = dynamic_cast<const beam_variables::ImuBiasGyro3DStamped&>(
          graph_msg->getVariable(bias_gyroscope_.uuid()));
      bias_acceleration_ = dynamic_cast<const beam_variables::ImuBiasAccel3DStamped&>(
          graph_msg->getVariable(bias_acceleration_.uuid()));
      updates_++;
      return true;
    }
    return false;
  }

  inline int Updates() const { return updates_; }

  ros::Time Stamp() const { return stamp_; }

  fuse_variables::Orientation3DStamped Orientation() const {
    return orientation_;
  }

  Eigen::Quaterniond OrientationQuat() const {
    Eigen::Quaterniond q(orientation_.w(), orientation_.x(), orientation_.y(), 
                         orientation_.z());
    return q;
  }

  fuse_variables::Position3DStamped Position() const { return position_; }

  fuse_core::Vector3d PositionVec() const { 
    fuse_core::Vector3d p(position_.x(), position_.y(), position_.z());
    return p; 
  }

  fuse_variables::VelocityLinear3DStamped Velocity() const { return velocity_; }

  fuse_core::Vector3d VelocityVec() const { 
    fuse_core::Vector3d v(velocity_.x(), velocity_.y(), velocity_.z());
    return v; 
  }

  beam_variables::ImuBiasGyro3DStamped BiasGyroscope() const {
    return bias_gyroscope_;
  }

  Eigen::Vector3d BiasGyroscopeVec() const { 
    Eigen::Vector3d bg(bias_gyroscope_.x(), bias_gyroscope_.y(), 
                           bias_gyroscope_.z());
    return bg; 
  }

  beam_variables::ImuBiasAccel3DStamped BiasAcceleration() const {
    return bias_acceleration_;
  }

  Eigen::Vector3d BiasAccelerationVec() const { 
    Eigen::Vector3d ba(bias_acceleration_.x(), bias_acceleration_.y(), 
                           bias_acceleration_.z());
    return ba; 
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

  void SetOrientation(const double* orientation) {
    orientation_.w() = orientation[0];
    orientation_.x() = orientation[1];
    orientation_.y() = orientation[2];
    orientation_.z() = orientation[3];
  }

  void SetPosition(const double& x, const double& y, const double& z) {
    position_.x() = x;
    position_.y() = y;
    position_.z() = z;
  }

  void SetPosition(const Eigen::Vector3d& position) {
    position_.x() = position[0];
    position_.y() = position[1];
    position_.z() = position[2];
  }

  void SetPosition(const double* position) {
    position_.x() = position[0];
    position_.y() = position[1];
    position_.z() = position[2];
  }

  void SetVelocity(const double& x, const double& y, const double& z) {
    velocity_.x() = x;
    velocity_.y() = y;
    velocity_.z() = z;
  }

  void SetVelocity(const Eigen::Vector3d& velocity) {
    velocity_.x() = velocity[0];
    velocity_.y() = velocity[1];
    velocity_.z() = velocity[2];
  }

  void SetVelocity(const double* velocity) {
    velocity_.x() = velocity[0];
    velocity_.y() = velocity[1];
    velocity_.z() = velocity[2];
  }  

  void SetBiasGyroscope(const double& x, const double& y, const double& z) {
    bias_gyroscope_.x() = x;
    bias_gyroscope_.y() = y;
    bias_gyroscope_.z() = z;
  }

  void SetBiasGyroscope(const Eigen::Vector3d& bias_gyroscope) {
    bias_gyroscope_.x() = bias_gyroscope[0];
    bias_gyroscope_.y() = bias_gyroscope[1];
    bias_gyroscope_.z() = bias_gyroscope[2];
  }

  void SetBiasGyroscope(const double* bias_gyroscope) {
    bias_gyroscope_.x() = bias_gyroscope[0];
    bias_gyroscope_.y() = bias_gyroscope[1];
    bias_gyroscope_.z() = bias_gyroscope[2];
  }

  void SetBiasAcceleration(const double& x, const double& y, const double& z) {
    bias_acceleration_.x() = x;
    bias_acceleration_.y() = y;
    bias_acceleration_.z() = z;
  }

  void SetBiasAcceleration(const Eigen::Vector3d& bias_acceleration) {
    bias_acceleration_.x() = bias_acceleration[0];
    bias_acceleration_.y() = bias_acceleration[1];
    bias_acceleration_.z() = bias_acceleration[2];
  }

  void SetBiasAcceleration(const double* bias_acceleration) {
    bias_acceleration_.x() = bias_acceleration[0];
    bias_acceleration_.y() = bias_acceleration[1];
    bias_acceleration_.z() = bias_acceleration[2];
  }

  void Print(std::ostream& stream = std::cout) const {
    stream << "  Stamp: " << stamp_ << "\n"
           << "  Number of Updates: " << updates_ << "\n"
           << "  Orientation:\n"
           << "  - w: " << orientation_.w() << "\n"
           << "  - x: " << orientation_.x() << "\n"
           << "  - y: " << orientation_.y() << "\n"
           << "  - z: " << orientation_.z() << "\n"
           << "  Position:\n"
           << "  - x: " << position_.x() << "\n"
           << "  - y: " << position_.y() << "\n"
           << "  - z: " << position_.z() << "\n"           
           << "  Velocity:\n"
           << "  - x: " << velocity_.x() << "\n"
           << "  - y: " << velocity_.y() << "\n"
           << "  - z: " << velocity_.z() << "\n"           
           << "  Bias Gyroscope:\n"
           << "  - x: " << bias_gyroscope_.x() << "\n"
           << "  - y: " << bias_gyroscope_.y() << "\n"
           << "  - z: " << bias_gyroscope_.z() << "\n"
           << "  Bias Acceleration:\n"
           << "  - x: " << bias_acceleration_.x() << "\n"
           << "  - y: " << bias_acceleration_.y() << "\n"
           << "  - z: " << bias_acceleration_.z() << "\n";
  }

private:
  void InstantiateFuseVariables() {
    orientation_ =
        fuse_variables::Orientation3DStamped(stamp_, fuse_core::uuid::NIL);  
    position_ = fuse_variables::Position3DStamped(stamp_, fuse_core::uuid::NIL);        
    velocity_ =
        fuse_variables::VelocityLinear3DStamped(stamp_, fuse_core::uuid::NIL); 
    bias_gyroscope_ =
        beam_variables::ImuBiasGyro3DStamped(stamp_, fuse_core::uuid::NIL);                  
    bias_acceleration_ =
        beam_variables::ImuBiasAccel3DStamped(stamp_, fuse_core::uuid::NIL);
  }

  int updates_{0};
  ros::Time stamp_;
  fuse_variables::Orientation3DStamped orientation_;
  fuse_variables::Position3DStamped position_;
  fuse_variables::VelocityLinear3DStamped velocity_;
  beam_variables::ImuBiasGyro3DStamped bias_gyroscope_;
  beam_variables::ImuBiasAccel3DStamped bias_acceleration_;
};

}}  // namespace beam_models::frame_to_frame
