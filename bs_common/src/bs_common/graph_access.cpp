#include <bs_common/graph_access.h>

#include <fuse_variables/point_3d_landmark.h>

namespace bs_common {

int GetNumberOfConstraints(
    const fuse_core::Transaction::SharedPtr& transaction) {
  if (transaction == nullptr) { return 0; }

  int counter = 0;
  auto added_constraints = transaction->addedConstraints();
  for (auto iter = added_constraints.begin(); iter != added_constraints.end();
       iter++) {
    counter++;
  }
  return counter;
}

int GetNumberOfVariables(const fuse_core::Transaction::SharedPtr& transaction) {
  if (transaction == nullptr) { return 0; }

  int counter = 0;
  auto added_variables = transaction->addedVariables();
  for (auto iter = added_variables.begin(); iter != added_variables.end();
       iter++) {
    counter++;
  }
  return counter;
}

bs_variables::GyroscopeBias3DStamped::SharedPtr
    GetGryoscopeBias(fuse_core::Graph::ConstSharedPtr graph,
                     const ros::Time& stamp) {
  auto gyro_bias = bs_variables::GyroscopeBias3DStamped::make_shared();
  const auto bg_uuid =
      fuse_core::uuid::generate(gyro_bias->type(), stamp, fuse_core::uuid::NIL);
  try {
    *gyro_bias = dynamic_cast<const bs_variables::GyroscopeBias3DStamped&>(
        graph->getVariable(bg_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return gyro_bias;
}

bs_variables::AccelerationBias3DStamped::SharedPtr
    GetAccelBias(fuse_core::Graph::ConstSharedPtr graph,
                 const ros::Time& stamp) {
  auto accel_bias = bs_variables::AccelerationBias3DStamped::make_shared();
  const auto ba_uuid = fuse_core::uuid::generate(accel_bias->type(), stamp,
                                                 fuse_core::uuid::NIL);
  try {
    *accel_bias = dynamic_cast<const bs_variables::AccelerationBias3DStamped&>(
        graph->getVariable(ba_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return accel_bias;
}

fuse_variables::Position3DStamped::SharedPtr
    GetPosition(fuse_core::Graph::ConstSharedPtr graph,
                const ros::Time& stamp) {
  auto position = fuse_variables::Position3DStamped::make_shared();
  const auto pos_uuid =
      fuse_core::uuid::generate(position->type(), stamp, fuse_core::uuid::NIL);
  try {
    *position = dynamic_cast<const fuse_variables::Position3DStamped&>(
        graph->getVariable(pos_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return position;
}

fuse_variables::Orientation3DStamped::SharedPtr
    GetOrientation(fuse_core::Graph::ConstSharedPtr graph,
                   const ros::Time& stamp) {
  auto orientation = fuse_variables::Orientation3DStamped::make_shared();
  const auto or_uuid = fuse_core::uuid::generate(orientation->type(), stamp,
                                                 fuse_core::uuid::NIL);
  try {
    *orientation = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
        graph->getVariable(or_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return orientation;
}

fuse_variables::VelocityLinear3DStamped::SharedPtr
    GetVelocity(fuse_core::Graph::ConstSharedPtr graph,
                const ros::Time& stamp) {
  auto velocity = fuse_variables::VelocityLinear3DStamped::make_shared();
  const auto vel_uuid =
      fuse_core::uuid::generate(velocity->type(), stamp, fuse_core::uuid::NIL);
  try {
    *velocity = dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(
        graph->getVariable(vel_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return velocity;
}

fuse_variables::VelocityAngular3DStamped::SharedPtr
    GetAngularVelocity(fuse_core::Graph::ConstSharedPtr graph,
                       const ros::Time& stamp) {
  auto velocity = fuse_variables::VelocityAngular3DStamped::make_shared();
  const auto vel_uuid =
      fuse_core::uuid::generate(velocity->type(), stamp, fuse_core::uuid::NIL);
  try {
    *velocity = dynamic_cast<const fuse_variables::VelocityAngular3DStamped&>(
        graph->getVariable(vel_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return velocity;
}

fuse_variables::AccelerationLinear3DStamped::SharedPtr
    GetLinearAcceleration(fuse_core::Graph::ConstSharedPtr graph,
                          const ros::Time& stamp) {
  auto velocity = fuse_variables::AccelerationLinear3DStamped::make_shared();
  const auto vel_uuid =
      fuse_core::uuid::generate(velocity->type(), stamp, fuse_core::uuid::NIL);
  try {
    *velocity =
        dynamic_cast<const fuse_variables::AccelerationLinear3DStamped&>(
            graph->getVariable(vel_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return velocity;
}

std::set<ros::Time> CurrentTimestamps(fuse_core::Graph::ConstSharedPtr graph) {
  std::set<ros::Time> times;
  for (auto& var : graph->getVariables()) {
    auto position = fuse_variables::Position3DStamped::make_shared();
    if (var.type() != position->type()) { continue; }
    *position = dynamic_cast<const fuse_variables::Position3DStamped&>(var);
    times.insert(position->stamp());
  }
  return times;
}

std::set<uint64_t> CurrentLandmarkIDs(fuse_core::Graph::ConstSharedPtr graph) {
  std::set<uint64_t> ids;
  for (auto& var : graph->getVariables()) {
    auto landmark = fuse_variables::Point3DLandmark::make_shared();
    if (var.type() != landmark->type()) { continue; }
    *landmark = dynamic_cast<const fuse_variables::Point3DLandmark&>(var);
    ids.insert(landmark->id());
  }
  return ids;
}

} // namespace bs_common
