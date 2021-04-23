#pragma once

#include <fuse_core/graph.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>

#include <beam_variables/imu_bias_stamped.h>

namespace beam_models { namespace frame_to_frame {

class ImuState {
public:
  ImuState() = default;

  ImuState(const ros::Time& time, const Eigen::Matrix4d& T_WORLD_IMU,
           const fuse_core::Vector3d& velocity,
           const fuse_core::Vector3d& bias_acceleration,
           const fuse_core::Vector3d& bias_gyroscope)
      : stamp_(time), T_WORLD_IMU_initial_(T_WORLD_IMU) {
    // create fuse variables
    position_ = fuse_variables::Position3DStamped(time, fuse_core::uuid::NIL);
    velocity_ =
        fuse_variables::VelocityLinear3DStamped(time, fuse_core::uuid::NIL);
    orientation_ =
        fuse_variables::Orientation3DStamped(time, fuse_core::uuid::NIL);
    bias_acceleration_ =
        beam_variables::ImuBiasStamped(time, fuse_core::uuid::NIL);
    bias_gyroscope_ =
        beam_variables::ImuBiasStamped(time, fuse_core::uuid::NIL);

    // add transform
    position_.x() = T_WORLD_IMU(0, 3);
    position_.y() = T_WORLD_IMU(1, 3);
    position_.z() = T_WORLD_IMU(2, 3);
    Eigen::Matrix3d R = T_WORLD_IMU.block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);
    orientation_.w() = q.w();
    orientation_.x() = q.x();
    orientation_.y() = q.y();
    orientation_.z() = q.z();

    // populate fields of remaining variables
    velocity_.x() = velocity[0];
    velocity_.y() = velocity[1];
    velocity_.z() = velocity[2];
    bias_acceleration_.x() = bias_acceleration[0];
    bias_acceleration_.y() = bias_acceleration[1];
    bias_acceleration_.z() = bias_acceleration[2];
    bias_gyroscope_.x() = bias_gyroscope[0];
    bias_gyroscope_.y() = bias_gyroscope[1];
    bias_gyroscope_.z() = bias_gyroscope[2];
  }

  ImuState(const ros::Time& time, const fuse_core::Vector3d& position,
           const Eigen::Quaterniond& orientation,
           const fuse_core::Vector3d& velocity,
           const fuse_core::Vector3d& bias_acceleration,
           const fuse_core::Vector3d& bias_gyroscope)
      : stamp_(time) {
    // create fuse variables
    position_ = fuse_variables::Position3DStamped(time, fuse_core::uuid::NIL);
    velocity_ =
        fuse_variables::VelocityLinear3DStamped(time, fuse_core::uuid::NIL);
    orientation_ =
        fuse_variables::Orientation3DStamped(time, fuse_core::uuid::NIL);
    bias_acceleration_ =
        beam_variables::ImuBiasStamped(time, fuse_core::uuid::NIL);
    bias_gyroscope_ =
        beam_variables::ImuBiasStamped(time, fuse_core::uuid::NIL);

    // populate fields
    position_.x() = position[0];
    position_.y() = position[1];
    position_.z() = position[2];
    orientation_.w() = orientation.w();
    orientation_.x() = orientation.x();
    orientation_.y() = orientation.y();
    orientation_.z() = orientation.z();
    velocity_.x() = velocity[0];
    velocity_.y() = velocity[1];
    velocity_.z() = velocity[2];
    bias_acceleration_.x() = bias_acceleration[0];
    bias_acceleration_.y() = bias_acceleration[1];
    bias_acceleration_.z() = bias_acceleration[2];
    bias_gyroscope_.x() = bias_gyroscope[0];
    bias_gyroscope_.y() = bias_gyroscope[1];
    bias_gyroscope_.z() = bias_gyroscope[2];
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

  Eigen::Matrix4d T_WORLD_IMU() const {
    Eigen::Matrix4d T_WORLD_IMU{Eigen::Matrix4d::Identity()};
    Eigen::Quaterniond q(orientation_.w(), orientation_.x(), orientation_.y(),
                         orientation_.z());
    T_WORLD_IMU.block(0, 3, 3, 1) =
        Eigen::Vector3d{position_.x(), position_.y(), position_.z()};
    T_WORLD_IMU.block(0, 0, 3, 3) = q.toRotationMatrix();
    return T_WORLD_IMU;
  }

  inline int Updates() const { return updates_; }

  const Eigen::Matrix4d T_WORLD_IMU_INIT() const {
    return T_WORLD_IMU_initial_;
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

  Eigen::Matrix<double, 16, 1> State() const {
    Eigen::Matrix<double, 16, 1> state;
    state << position_.x(), position_.y(), position_.z(), orientation_.w(),
        orientation_.x(), orientation_.y(), orientation_.z(), velocity_.x(),
        velocity_.y(), velocity_.z(), bias_acceleration_.x(),
        bias_acceleration_.y(), bias_acceleration_.z(), bias_gyroscope_.x(),
        bias_gyroscope_.y(), bias_gyroscope_.z();
    return state;
  }

  void SetPosition(const double& x, const double& y, const double& z) {
    position_.x() = x;
    position_.y() = y;
    position_.z() = z;
  }

  void SetVelocity(const double& x, const double& y, const double& z) {
    velocity_.x() = x;
    velocity_.y() = y;
    velocity_.z() = z;
  }

  void SetOrientation(const double& w, const double& x, const double& y,
                      const double& z) {
    orientation_.w() = w;
    orientation_.x() = x;
    orientation_.y() = y;
    orientation_.z() = z;
  }

  void SetBiasAcceleration(const double& x, const double& y, const double& z) {
    bias_acceleration_.x() = x;
    bias_acceleration_.y() = y;
    bias_acceleration_.z() = z;
  }

  void SetBiasGyroscope(const double& x, const double& y, const double& z) {
    bias_gyroscope_.x() = x;
    bias_gyroscope_.y() = y;
    bias_gyroscope_.z() = z;
  }

  void SetState(const double& p_x, const double& p_y, const double& p_z,
                const double& v_x, const double& v_y, const double& v_z,
                const double& o_w, const double& o_x, const double& o_y,
                const double& o_z, const double& ba_x, const double& ba_y,
                const double& ba_z, const double& bg_x, const double& bg_y,
                const double& bg_z) {
    position_.x() = p_x;
    position_.y() = p_y;
    position_.z() = p_z;
    velocity_.x() = v_x;
    velocity_.y() = v_y;
    velocity_.z() = v_z;
    orientation_.w() = o_w;
    orientation_.x() = o_x;
    orientation_.y() = o_y;
    orientation_.z() = o_z;
    bias_acceleration_.x() = ba_x;
    bias_acceleration_.y() = ba_y;
    bias_acceleration_.z() = ba_z;
    bias_gyroscope_.x() = bg_x;
    bias_gyroscope_.y() = bg_y;
    bias_gyroscope_.z() = bg_z;
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
           << "  - x: " << orientation_.x() << "\n"
           << "  - y: " << orientation_.y() << "\n"
           << "  - z: " << orientation_.z() << "\n"
           << "  - w: " << orientation_.w() << "\n"
           << "  Bias Acceleration:\n"
           << "  - x: " << bias_acceleration_.x() << "\n"
           << "  - y: " << bias_acceleration_.y() << "\n"
           << "  - z: " << bias_acceleration_.z() << "\n"
           << "  Bias Gyroscope:\n"
           << "  - x: " << bias_gyroscope_.x() << "\n"
           << "  - y: " << bias_gyroscope_.y() << "\n"
           << "  - z: " << bias_gyroscope_.z() << "\n";
  }

protected:
  ros::Time stamp_;
  int updates_{0};
  fuse_variables::Position3DStamped position_;
  fuse_variables::VelocityLinear3DStamped velocity_;
  fuse_variables::Orientation3DStamped orientation_;
  beam_variables::ImuBiasStamped bias_acceleration_;
  beam_variables::ImuBiasStamped bias_gyroscope_;
  const Eigen::Matrix4d T_WORLD_IMU_initial_;
};

}}  // namespace beam_models::frame_to_frame