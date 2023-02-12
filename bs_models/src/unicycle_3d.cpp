#include <bs_models/unicycle_3d.h>

#include <stdexcept>

#include <boost/range/size.hpp>

#include <fuse_variables/acceleration_linear_3d_stamped.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/stamped.h>
#include <fuse_variables/velocity_angular_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>
#include <pluginlib/class_list_macros.h>

#include <bs_constraints/motion/unicycle_3d_predict.h>
#include <bs_constraints/motion/unicycle_3d_state_kinematic_constraint.h>

// Register this motion model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(bs_models::Unicycle3D, fuse_core::MotionModel)

namespace bs_models {

static constexpr double EPSILON =
    1.0e-9;  //!< "Small" value used to check if state variables are effectively
             //!< zero

Unicycle3D::Unicycle3D()
    : fuse_core::AsyncMotionModel(1),
      buffer_length_(ros::DURATION_MAX),
      device_id_(fuse_core::uuid::NIL),
      timestamp_manager_(&Unicycle3D::generateMotionModel, this,
                         ros::DURATION_MAX) {}

bool Unicycle3D::applyCallback(fuse_core::Transaction& transaction) {
  auto s =
      boost::range_detail::range_calculate_size(transaction.addedConstraints());
  auto v =
      boost::range_detail::range_calculate_size(transaction.addedVariables());

  try {
    // Now actually generate the motion model segments
    timestamp_manager_.query(transaction, false);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM_THROTTLE(
        10.0,
        "An error occurred while completing the motion model query. Error: "
            << e.what());
    return false;
  }

  return true;
}

void Unicycle3D::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) {
  updateStateHistoryEstimates(*graph, state_history_, buffer_length_);
}

void Unicycle3D::onInit() {
  std::vector<double> process_noise_diagonal;
  private_node_handle_.param("process_noise_diagonal", process_noise_diagonal,
                             process_noise_diagonal);

  if (process_noise_diagonal.size() != 15) {
    ROS_ERROR("Process noise diagonal must be of length 15. Length given: %zu",
              process_noise_diagonal.size());
    throw std::runtime_error("Process noise diagonal must be of length 15!");
  }

  process_noise_covariance_ =
      fuse_core::Vector15d(process_noise_diagonal.data()).asDiagonal();

  double buffer_length = 3.0;
  private_node_handle_.param("buffer_length", buffer_length, buffer_length);

  if (buffer_length < 0.0) {
    throw std::runtime_error("Invalid negative buffer length of " +
                             std::to_string(buffer_length) + " specified.");
  }

  buffer_length_ =
      (buffer_length == 0.0) ? ros::DURATION_MAX : ros::Duration(buffer_length);
  timestamp_manager_.bufferLength(buffer_length_);

  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);
}

void Unicycle3D::onStart() {
  timestamp_manager_.clear();
  state_history_.clear();
}

void Unicycle3D::generateMotionModel(
    const ros::Time& beginning_stamp, const ros::Time& ending_stamp,
    std::vector<fuse_core::Constraint::SharedPtr>& constraints,
    std::vector<fuse_core::Variable::SharedPtr>& variables) {
  StateHistoryElement base_state;
  ros::Time base_time;

  auto base_state_pair_it = state_history_.upper_bound(beginning_stamp);
  if (base_state_pair_it == state_history_.begin()) {
    ROS_WARN_STREAM_COND_NAMED(
        !state_history_.empty(), "UnicycleModel",
        "Unable to locate a state in this history "
        "with stamp <= "
            << beginning_stamp << ". Variables will all be initialized to 0.");
    base_time = beginning_stamp;
  } else {
    --base_state_pair_it;
    base_time = base_state_pair_it->first;
    base_state = base_state_pair_it->second;
  }

  StateHistoryElement state1;
  StateHistoryElement state2;

  // If the nearest state we had was before the beginning stamp, we need to
  // project that state to the beginning stamp
  if (base_time != beginning_stamp) {
    bs_constraints::motion::predict(
        base_state.pose, base_state.velocity_linear,
        base_state.velocity_angular, base_state.acceleration_linear,
        (beginning_stamp - base_time).toSec(), state1.pose,
        state1.velocity_linear, state1.velocity_angular,
        state1.acceleration_linear);
  } else {
    state1 = base_state;
  }

  const double dt = (ending_stamp - beginning_stamp).toSec();

  // Now predict to get an initial guess for the state at the ending stamp
  bs_constraints::motion::predict(
      state1.pose, state1.velocity_linear, state1.velocity_angular,
      state1.acceleration_linear, dt, state2.pose, state2.velocity_linear,
      state2.velocity_angular, state2.acceleration_linear);

  // Define the fuse variables required for this constraint
  auto position1 = fuse_variables::Position3DStamped::make_shared(
      beginning_stamp, device_id_);
  auto orientation1 = fuse_variables::Orientation3DStamped::make_shared(
      beginning_stamp, device_id_);
  auto velocity_linear1 = fuse_variables::VelocityLinear3DStamped::make_shared(
      beginning_stamp, device_id_);
  auto velocity_angular1 =
      fuse_variables::VelocityAngular3DStamped::make_shared(beginning_stamp,
                                                            device_id_);
  auto acceleration_linear1 =
      fuse_variables::AccelerationLinear3DStamped::make_shared(beginning_stamp,
                                                               device_id_);
  auto position2 =
      fuse_variables::Position3DStamped::make_shared(ending_stamp, device_id_);
  auto orientation2 = fuse_variables::Orientation3DStamped::make_shared(
      ending_stamp, device_id_);
  auto velocity_linear2 = fuse_variables::VelocityLinear3DStamped::make_shared(
      ending_stamp, device_id_);
  auto velocity_angular2 =
      fuse_variables::VelocityAngular3DStamped::make_shared(ending_stamp,
                                                            device_id_);
  auto acceleration_linear2 =
      fuse_variables::AccelerationLinear3DStamped::make_shared(ending_stamp,
                                                               device_id_);

  position1->data()[fuse_variables::Position3DStamped::X] =
      state1.pose.getOrigin().x();
  position1->data()[fuse_variables::Position3DStamped::Y] =
      state1.pose.getOrigin().y();
  position1->data()[fuse_variables::Position3DStamped::Z] =
      state1.pose.getOrigin().z();

  double roll1, pitch1, yaw1;
  state1.pose.getBasis().getRPY(roll1, pitch1, yaw1);
  orientation1->data()[fuse_variables::Orientation3DStamped::X] =
      state1.pose.getRotation().x();
  orientation1->data()[fuse_variables::Orientation3DStamped::Y] =
      state1.pose.getRotation().y();
  orientation1->data()[fuse_variables::Orientation3DStamped::Z] =
      state1.pose.getRotation().z();
  orientation1->data()[fuse_variables::Orientation3DStamped::W] =
      state1.pose.getRotation().w();

  velocity_linear1->data()[fuse_variables::VelocityLinear3DStamped::X] =
      state1.velocity_linear.x();
  velocity_linear1->data()[fuse_variables::VelocityLinear3DStamped::Y] =
      state1.velocity_linear.y();
  velocity_linear1->data()[fuse_variables::VelocityLinear3DStamped::Z] =
      state1.velocity_linear.z();

  velocity_angular1->data()[fuse_variables::VelocityAngular3DStamped::ROLL] =
      state1.velocity_angular.x();
  velocity_angular1->data()[fuse_variables::VelocityAngular3DStamped::PITCH] =
      state1.velocity_angular.y();
  velocity_angular1->data()[fuse_variables::VelocityAngular3DStamped::YAW] =
      state1.velocity_angular.z();

  acceleration_linear1->data()[fuse_variables::AccelerationLinear3DStamped::X] =
      state1.acceleration_linear.x();
  acceleration_linear1->data()[fuse_variables::AccelerationLinear3DStamped::Y] =
      state1.acceleration_linear.y();
  acceleration_linear1->data()[fuse_variables::AccelerationLinear3DStamped::Z] =
      state1.acceleration_linear.z();

  position2->data()[fuse_variables::Position3DStamped::X] =
      state2.pose.getOrigin().x();
  position2->data()[fuse_variables::Position3DStamped::Y] =
      state2.pose.getOrigin().y();
  position2->data()[fuse_variables::Position3DStamped::Z] =
      state2.pose.getOrigin().z();

  double roll2, pitch2, yaw2;
  state2.pose.getBasis().getRPY(roll2, pitch2, yaw2);
  orientation2->data()[fuse_variables::Orientation3DStamped::X] =
      state2.pose.getRotation().x();
  orientation2->data()[fuse_variables::Orientation3DStamped::Y] =
      state2.pose.getRotation().y();
  orientation2->data()[fuse_variables::Orientation3DStamped::Z] =
      state2.pose.getRotation().z();
  orientation2->data()[fuse_variables::Orientation3DStamped::W] =
      state2.pose.getRotation().w();

  velocity_linear2->data()[fuse_variables::VelocityLinear3DStamped::X] =
      state2.velocity_linear.x();
  velocity_linear2->data()[fuse_variables::VelocityLinear3DStamped::Y] =
      state2.velocity_linear.y();
  velocity_linear2->data()[fuse_variables::VelocityLinear3DStamped::Z] =
      state2.velocity_linear.z();

  velocity_angular2->data()[fuse_variables::VelocityAngular3DStamped::ROLL] =
      state2.velocity_angular.x();
  velocity_angular2->data()[fuse_variables::VelocityAngular3DStamped::PITCH] =
      state2.velocity_angular.y();
  velocity_angular2->data()[fuse_variables::VelocityAngular3DStamped::YAW] =
      state2.velocity_angular.z();

  acceleration_linear2->data()[fuse_variables::AccelerationLinear3DStamped::X] =
      state2.acceleration_linear.x();
  acceleration_linear2->data()[fuse_variables::AccelerationLinear3DStamped::Y] =
      state2.acceleration_linear.y();
  acceleration_linear2->data()[fuse_variables::AccelerationLinear3DStamped::Z] =
      state2.acceleration_linear.z();

  state1.position_uuid = position1->uuid();
  state1.orientation_uuid = orientation1->uuid();
  state1.vel_linear_uuid = velocity_linear1->uuid();
  state1.vel_angular_uuid = velocity_angular1->uuid();
  state1.acc_linear_uuid = acceleration_linear1->uuid();

  state2.position_uuid = position2->uuid();
  state2.orientation_uuid = orientation2->uuid();
  state2.vel_linear_uuid = velocity_linear2->uuid();
  state2.vel_angular_uuid = velocity_angular2->uuid();
  state2.acc_linear_uuid = acceleration_linear2->uuid();

  state_history_.emplace(beginning_stamp, std::move(state1));
  state_history_.emplace(ending_stamp, std::move(state2));

  // Create the constraints for this motion model segment
  auto constraint =
      bs_constraints::motion::Unicycle3DStateKinematicConstraint::make_shared(
          name(), *position1, *orientation1, *velocity_linear1,
          *velocity_angular1, *acceleration_linear1, *position2, *orientation2,
          *velocity_linear2, *velocity_angular2, *acceleration_linear2,
          process_noise_covariance_ * dt);

  // Update the output variables
  constraints.push_back(constraint);
  variables.push_back(position1);
  variables.push_back(orientation1);
  variables.push_back(velocity_linear1);
  variables.push_back(velocity_angular1);
  variables.push_back(acceleration_linear1);
  variables.push_back(position2);
  variables.push_back(orientation2);
  variables.push_back(velocity_linear2);
  variables.push_back(velocity_angular2);
  variables.push_back(acceleration_linear2);
}

void Unicycle3D::updateStateHistoryEstimates(
    const fuse_core::Graph& graph, StateHistory& state_history,
    const ros::Duration& buffer_length) {
  if (state_history.empty()) {
    return;
  }

  ros::Time expiration_time;

  // ROS can't handle negative times
  if (state_history.rbegin()->first.toSec() < buffer_length.toSec()) {
    expiration_time = ros::Time(0);
  } else {
    expiration_time = state_history.rbegin()->first - buffer_length;
  }

  auto current_iter = state_history.begin();

  // always keep at least one entry in the buffer
  while (state_history.size() > 1 && current_iter->first < expiration_time) {
    current_iter = state_history.erase(current_iter);
  }

  // Update the states in the state history with information from the graph
  // If a state is not in the graph yet, predict the state in question from the
  // closest previous state
  for (auto current_iter = state_history.begin();
       current_iter != state_history.end(); ++current_iter) {
    const auto& current_stamp = current_iter->first;
    auto& current_state = current_iter->second;
    if (graph.variableExists(current_state.position_uuid) &&
        graph.variableExists(current_state.orientation_uuid) &&
        graph.variableExists(current_state.vel_linear_uuid) &&
        graph.variableExists(current_state.vel_angular_uuid) &&
        graph.variableExists(current_state.acc_linear_uuid)) {
      // This pose does exist in the graph. Update it directly.
      const auto& position = graph.getVariable(current_state.position_uuid);
      const auto& orientation =
          graph.getVariable(current_state.orientation_uuid);
      const auto& vel_linear = graph.getVariable(current_state.vel_linear_uuid);
      const auto& vel_angular =
          graph.getVariable(current_state.vel_angular_uuid);
      const auto& acc_linear = graph.getVariable(current_state.acc_linear_uuid);

      current_state.pose.setOrigin(
          tf2::Vector3{position.data()[fuse_variables::Position3DStamped::X],
                       position.data()[fuse_variables::Position3DStamped::Y],
                       position.data()[fuse_variables::Position3DStamped::Z]});

      // tf2::Quaternion assumes a (x,y,z,w) input
      current_state.pose.setRotation(tf2::Quaternion{
          orientation.data()[fuse_variables::Orientation3DStamped::X],
          orientation.data()[fuse_variables::Orientation3DStamped::Y],
          orientation.data()[fuse_variables::Orientation3DStamped::Z],
          orientation.data()[fuse_variables::Orientation3DStamped::W]});

      current_state.velocity_linear.setX(
          vel_linear.data()[fuse_variables::VelocityLinear3DStamped::X]);
      current_state.velocity_linear.setY(
          vel_linear.data()[fuse_variables::VelocityLinear3DStamped::Y]);
      current_state.velocity_linear.setZ(
          vel_linear.data()[fuse_variables::VelocityLinear3DStamped::Z]);

      current_state.velocity_angular.setX(
          vel_angular.data()[fuse_variables::VelocityAngular3DStamped::ROLL]);
      current_state.velocity_angular.setY(
          vel_angular.data()[fuse_variables::VelocityAngular3DStamped::PITCH]);
      current_state.velocity_angular.setZ(
          vel_angular.data()[fuse_variables::VelocityAngular3DStamped::YAW]);

      current_state.acceleration_linear.setX(
          acc_linear.data()[fuse_variables::AccelerationLinear3DStamped::X]);
      current_state.acceleration_linear.setY(
          acc_linear.data()[fuse_variables::AccelerationLinear3DStamped::Y]);
      current_state.acceleration_linear.setZ(
          acc_linear.data()[fuse_variables::AccelerationLinear3DStamped::Z]);

    } else if (current_iter != state_history.begin()) {
      auto previous_iter = std::prev(current_iter);
      const auto& previous_stamp = previous_iter->first;
      const auto& previous_state = previous_iter->second;

      // This state is not in the graph yet, so we can't update/correct the
      // value in our state history. However, the state *before* this one may
      // have been corrected (or one of its predecessors may have been), so we
      // can use that corrected value, along with our prediction logic, to
      // provide a more accurate update to this state.
      bs_constraints::motion::predict(
          previous_state.pose, previous_state.velocity_linear,
          previous_state.velocity_angular, previous_state.acceleration_linear,
          (current_stamp - previous_stamp).toSec(), current_state.pose,
          current_state.velocity_linear, current_state.velocity_angular,
          current_state.acceleration_linear);

      double roll1, pitch1, yaw1;
      previous_state.pose.getBasis().getRPY(roll1, pitch1, yaw1);
      double roll2, pitch2, yaw2;
      current_state.pose.getBasis().getRPY(roll2, pitch2, yaw2);
    }
  }
}

}  // namespace bs_models
