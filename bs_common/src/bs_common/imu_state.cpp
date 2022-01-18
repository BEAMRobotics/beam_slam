#include <bs_common/imu_state.h>

namespace bs_common {

ImuState::ImuState(const ros::Time &time) : stamp_(time) {
  InstantiateVariables();
  SetOrientation(1, 0, 0, 0);
  SetPosition(0, 0, 0);
  SetVelocity(0, 0, 0);
  SetGyroBias(0, 0, 0);
  SetAccelBias(0, 0, 0);
}

ImuState::ImuState(const ros::Time &time, const Eigen::Quaterniond &orientation,
                   const Eigen::Vector3d &position,
                   const Eigen::Vector3d &velocity)
    : stamp_(time) {
  InstantiateVariables();
  SetOrientation(orientation);
  SetPosition(position);
  SetVelocity(velocity);
  SetGyroBias(0, 0, 0);
  SetAccelBias(0, 0, 0);
}

ImuState::ImuState(const ros::Time &time, const Eigen::Quaterniond &orientation,
         const Eigen::Vector3d &position,
         const bs_common::PreIntegrator &preint): stamp_(time) {
  InstantiateVariables();
  SetOrientation(orientation);
  SetPosition(position);
  SetPreintegrator(preint);
  SetVelocity(0, 0, 0);
  SetGyroBias(0, 0, 0);
  SetAccelBias(0, 0, 0);
}

ImuState::ImuState(const ros::Time &time, const Eigen::Quaterniond &orientation,
                   const Eigen::Vector3d &position,
                   const Eigen::Vector3d &velocity,
                   const Eigen::Vector3d &gyrobias,
                   const Eigen::Vector3d &accelbias)
    : stamp_(time) {
  InstantiateVariables();
  SetOrientation(orientation);
  SetPosition(position);
  SetVelocity(velocity);
  SetGyroBias(gyrobias);
  SetAccelBias(accelbias);
}

bool ImuState::Update(const fuse_core::Graph::SharedPtr &graph_msg) {
  if (graph_msg->variableExists(orientation_->uuid()) &&
      graph_msg->variableExists(position_->uuid()) &&
      graph_msg->variableExists(velocity_->uuid()) &&
      graph_msg->variableExists(gyrobias_->uuid()) &&
      graph_msg->variableExists(accelbias_->uuid())) {
    *orientation_ = dynamic_cast<const fuse_variables::Orientation3DStamped &>(
        graph_msg->getVariable(orientation_->uuid()));
    *position_ = dynamic_cast<const fuse_variables::Position3DStamped &>(
        graph_msg->getVariable(position_->uuid()));
    *velocity_ = dynamic_cast<const fuse_variables::VelocityLinear3DStamped &>(
        graph_msg->getVariable(velocity_->uuid()));
    *gyrobias_ = dynamic_cast<const bs_variables::GyroscopeBias3DStamped &>(
        graph_msg->getVariable(gyrobias_->uuid()));
    *accelbias_ = dynamic_cast<const bs_variables::AccelerationBias3DStamped &>(
        graph_msg->getVariable(accelbias_->uuid()));
    updates_++;
    return true;
  }
  return false;
}

int ImuState::Updates() const { return updates_; }

ros::Time ImuState::Stamp() const { return stamp_; }

fuse_variables::Orientation3DStamped ImuState::Orientation() const {
  return *orientation_;
}

Eigen::Quaterniond ImuState::OrientationQuat() const {
  Eigen::Quaterniond q(orientation_->w(), orientation_->x(), orientation_->y(),
                       orientation_->z());
  return q.normalized();
}

Eigen::Matrix3d ImuState::OrientationMat() const {
  Eigen::Matrix3d R = OrientationQuat().toRotationMatrix();
  return R;
}

fuse_variables::Position3DStamped ImuState::Position() const {
  return *position_;
}

Eigen::Vector3d ImuState::PositionVec() const {
  Eigen::Vector3d p(position_->x(), position_->y(), position_->z());
  return p;
}

fuse_variables::VelocityLinear3DStamped ImuState::Velocity() const {
  return *velocity_;
}

Eigen::Vector3d ImuState::VelocityVec() const {
  Eigen::Vector3d v(velocity_->x(), velocity_->y(), velocity_->z());
  return v;
}

bs_variables::GyroscopeBias3DStamped ImuState::GyroBias() const {
  return *gyrobias_;
}

Eigen::Vector3d ImuState::GyroBiasVec() const {
  Eigen::Vector3d bg(gyrobias_->x(), gyrobias_->y(), gyrobias_->z());
  return bg;
}

bs_variables::AccelerationBias3DStamped ImuState::AccelBias() const {
  return *accelbias_;
}

Eigen::Vector3d ImuState::AccelBiasVec() const {
  Eigen::Vector3d ba(accelbias_->x(), accelbias_->y(), accelbias_->z());
  return ba;
}

std::shared_ptr<bs_common::PreIntegrator> ImuState::GetPreintegrator() {
  return preint_;
}

void ImuState::SetPreintegrator(const bs_common::PreIntegrator &preint) {
  *preint_ = preint;
}

void ImuState::SetOrientation(const double &w, const double &x, const double &y,
                              const double &z) {
  orientation_->w() = w;
  orientation_->x() = x;
  orientation_->y() = y;
  orientation_->z() = z;
}

void ImuState::SetOrientation(const Eigen::Quaterniond &orientation) {
  orientation_->w() = orientation.w();
  orientation_->x() = orientation.x();
  orientation_->y() = orientation.y();
  orientation_->z() = orientation.z();
}

void ImuState::SetOrientation(const double *orientation) {
  orientation_->w() = orientation[0];
  orientation_->x() = orientation[1];
  orientation_->y() = orientation[2];
  orientation_->z() = orientation[3];
}

void ImuState::SetPosition(const double &x, const double &y, const double &z) {
  position_->x() = x;
  position_->y() = y;
  position_->z() = z;
}

void ImuState::SetPosition(const Eigen::Vector3d &position) {
  position_->x() = position[0];
  position_->y() = position[1];
  position_->z() = position[2];
}

void ImuState::SetPosition(const double *position) {
  position_->x() = position[0];
  position_->y() = position[1];
  position_->z() = position[2];
}

void ImuState::SetVelocity(const double &x, const double &y, const double &z) {
  velocity_->x() = x;
  velocity_->y() = y;
  velocity_->z() = z;
}

void ImuState::SetVelocity(const Eigen::Vector3d &velocity) {
  velocity_->x() = velocity[0];
  velocity_->y() = velocity[1];
  velocity_->z() = velocity[2];
}

void ImuState::SetVelocity(const double *velocity) {
  velocity_->x() = velocity[0];
  velocity_->y() = velocity[1];
  velocity_->z() = velocity[2];
}

void ImuState::SetGyroBias(const double &x, const double &y, const double &z) {
  gyrobias_->x() = x;
  gyrobias_->y() = y;
  gyrobias_->z() = z;
}

void ImuState::SetGyroBias(const Eigen::Vector3d &gyrobias) {
  gyrobias_->x() = gyrobias[0];
  gyrobias_->y() = gyrobias[1];
  gyrobias_->z() = gyrobias[2];
}

void ImuState::SetGyroBias(const double *gyrobias) {
  gyrobias_->x() = gyrobias[0];
  gyrobias_->y() = gyrobias[1];
  gyrobias_->z() = gyrobias[2];
}

void ImuState::SetAccelBias(const double &x, const double &y, const double &z) {
  accelbias_->x() = x;
  accelbias_->y() = y;
  accelbias_->z() = z;
}

void ImuState::SetAccelBias(const Eigen::Vector3d &accelbias) {
  accelbias_->x() = accelbias[0];
  accelbias_->y() = accelbias[1];
  accelbias_->z() = accelbias[2];
}

void ImuState::SetAccelBias(const double *accelbias) {
  accelbias_->x() = accelbias[0];
  accelbias_->y() = accelbias[1];
  accelbias_->z() = accelbias[2];
}

void ImuState::Print(std::ostream &stream) const {
  stream << "  Stamp: " << stamp_ << "\n"
         << "  Number of Updates: " << updates_ << "\n"
         << "  Orientation:\n"
         << "  - w: " << orientation_->w() << "\n"
         << "  - x: " << orientation_->x() << "\n"
         << "  - y: " << orientation_->y() << "\n"
         << "  - z: " << orientation_->z() << "\n"
         << "  Position:\n"
         << "  - x: " << position_->x() << "\n"
         << "  - y: " << position_->y() << "\n"
         << "  - z: " << position_->z() << "\n"
         << "  Velocity:\n"
         << "  - x: " << velocity_->x() << "\n"
         << "  - y: " << velocity_->y() << "\n"
         << "  - z: " << velocity_->z() << "\n"
         << "  Gyroscope Bias:\n"
         << "  - x: " << gyrobias_->x() << "\n"
         << "  - y: " << gyrobias_->y() << "\n"
         << "  - z: " << gyrobias_->z() << "\n"
         << "  Acceleration Bias:\n"
         << "  - x: " << accelbias_->x() << "\n"
         << "  - y: " << accelbias_->y() << "\n"
         << "  - z: " << accelbias_->z() << "\n";
}

void ImuState::InstantiateVariables() {
  orientation_ = std::make_shared<fuse_variables::Orientation3DStamped>(
      stamp_, fuse_core::uuid::NIL);
  position_ = std::make_shared<fuse_variables::Position3DStamped>(
      stamp_, fuse_core::uuid::NIL);
  velocity_ = std::make_shared<fuse_variables::VelocityLinear3DStamped>(
      stamp_, fuse_core::uuid::NIL);
  gyrobias_ = std::make_shared<bs_variables::GyroscopeBias3DStamped>(
      stamp_, fuse_core::uuid::NIL);
  accelbias_ = std::make_shared<bs_variables::AccelerationBias3DStamped>(
      stamp_, fuse_core::uuid::NIL);
  preint_ = std::make_shared<bs_common::PreIntegrator>();
}

} // namespace bs_common
