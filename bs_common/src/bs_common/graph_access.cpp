#include <bs_common/graph_access.h>

#include <fuse_variables/point_3d_landmark.h>

#include <bs_variables/inverse_depth_landmark.h>

namespace bs_common {

std::map<int64_t, ImuBiases>
    GetImuBiasesFromGraph(const fuse_core::Graph& graph) {
  std::map<int64_t, ImuBiases> biases;
  const auto var_range = graph.getVariables();
  for (auto it = var_range.begin(); it != var_range.end(); it++) {
    if (it->type() == "bs_variables::GyroscopeBias3DStamped") {
      auto v = dynamic_cast<const bs_variables::GyroscopeBias3DStamped&>(*it);
      int64_t t_ns = v.stamp().toNSec();
      auto iter = biases.find(t_ns);
      if (iter == biases.end()) {
        ImuBiases b;
        b.g_x = v.x();
        b.g_y = v.y();
        b.g_z = v.z();
        biases.emplace(t_ns, b);
      } else {
        iter->second.g_x = v.x();
        iter->second.g_y = v.y();
        iter->second.g_z = v.z();
      }
    } else if (it->type() == "bs_variables::AccelerationBias3DStamped") {
      auto v =
          dynamic_cast<const bs_variables::AccelerationBias3DStamped&>(*it);
      int64_t t_ns = v.stamp().toNSec();
      auto iter = biases.find(t_ns);
      if (iter == biases.end()) {
        ImuBiases b;
        b.a_x = v.x();
        b.a_y = v.y();
        b.a_z = v.z();
        biases.emplace(t_ns, b);
      } else {
        iter->second.a_x = v.x();
        iter->second.a_y = v.y();
        iter->second.a_z = v.z();
      }
    }
  }
  return biases;
}

void SaveGraphToTxtFile(fuse_core::Graph::ConstSharedPtr graph,
                        const std::string& txt_file_save_path) {
  std::ofstream outFile;
  outFile.open(txt_file_save_path);
  graph->print(outFile);
  outFile.close();
}

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
    auto inversedepth_landmark =
        bs_variables::InverseDepthLandmark::make_shared();
    if (var.type() == landmark->type()) {
      *landmark = dynamic_cast<const fuse_variables::Point3DLandmark&>(var);
      ids.insert(landmark->id());
    } else if (var.type() == inversedepth_landmark->type()) {
      *inversedepth_landmark =
          dynamic_cast<const bs_variables::InverseDepthLandmark&>(var);
      ids.insert(inversedepth_landmark->id());
    }
  }
  return ids;
}

fuse_variables::Point3DLandmark::SharedPtr
    GetLandmark(fuse_core::Graph::ConstSharedPtr graph, const uint64_t id) {
  auto lm = fuse_variables::Point3DLandmark::make_shared();
  auto lm_uuid = fuse_core::uuid::generate(lm->type(), id);
  try {
    *lm = dynamic_cast<const fuse_variables::Point3DLandmark&>(
        graph->getVariable(lm_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return lm;
}

bs_variables::InverseDepthLandmark::SharedPtr
    GetInverseDepthLandmark(fuse_core::Graph::ConstSharedPtr graph,
                            const uint64_t id) {
  auto lm = bs_variables::InverseDepthLandmark::make_shared();
  auto lm_uuid = fuse_core::uuid::generate(lm->type(), id);
  try {
    *lm = dynamic_cast<const bs_variables::InverseDepthLandmark&>(
        graph->getVariable(lm_uuid));
  } catch (const std::out_of_range& oor) { return nullptr; }
  return lm;
}

} // namespace bs_common
