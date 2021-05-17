#include <basalt/spline/se3_spline.h>
#include <gtest/gtest.h>
#include <fuse_core/constraint.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_graphs/hash_graph.h>

#include <beam_models/frame_to_frame/imu_preintegration.h>

using namespace beam_models::frame_to_frame;
using namespace beam_constraints::frame_to_frame;
using namespace beam_constraints::global;

static const double GRAVITY = 9.81;

int AddConstraints(const fuse_core::Transaction::SharedPtr& transaction,
                   fuse_graphs::HashGraph& graph) {
  int counter = 0;
  RelativeImuState3DStampedConstraint dummy_relative_constraint;
  AbsoluteImuState3DStampedConstraint dummy_absolute_constraint;
  auto added_constraints = transaction->addedConstraints();
  for (auto iter = added_constraints.begin(); iter != added_constraints.end();
       iter++) {
    if (iter->type() == dummy_relative_constraint.type()) {
      auto constraint =
          dynamic_cast<const RelativeImuState3DStampedConstraint&>(*iter);
      fuse_core::Constraint::SharedPtr constraint_ptr =
          RelativeImuState3DStampedConstraint::make_shared(constraint);
      graph.addConstraint(constraint_ptr);
      counter++;
    } else if (iter->type() == dummy_absolute_constraint.type()) {
      auto constraint =
          dynamic_cast<const AbsoluteImuState3DStampedConstraint&>(*iter);
      fuse_core::Constraint::SharedPtr constraint_ptr =
          AbsoluteImuState3DStampedConstraint::make_shared(constraint);
      graph.addConstraint(constraint_ptr);
      counter++;
    } else {
      return counter;
    }
  }
  return counter;
}

std::vector<fuse_core::UUID> AddVariables(
    const fuse_core::Transaction::SharedPtr& transaction,
    fuse_graphs::HashGraph& graph) {
  fuse_variables::Orientation3DStamped dummy_orientation;
  fuse_variables::Position3DStamped dummy_position;
  fuse_variables::VelocityLinear3DStamped dummy_velocity;
  beam_variables::ImuBiasGyro3DStamped dummy_imu_bias_gyro;
  beam_variables::ImuBiasAccel3DStamped dummy_imu_bias_accel;
  std::vector<fuse_core::UUID> uuids;
  auto added_variables = transaction->addedVariables();

  for (auto iter = added_variables.begin(); iter != added_variables.end();
       iter++) {
    if (iter->type() == dummy_orientation.type()) {
      auto var =
          dynamic_cast<const fuse_variables::Orientation3DStamped&>(*iter);
      fuse_core::Variable::SharedPtr var_ptr =
          fuse_variables::Orientation3DStamped::make_shared(var);
      graph.addVariable(var_ptr);
    } else if (iter->type() == dummy_position.type()) {
      auto var = dynamic_cast<const fuse_variables::Position3DStamped&>(*iter);
      fuse_core::Variable::SharedPtr var_ptr =
          fuse_variables::Position3DStamped::make_shared(var);
      graph.addVariable(var_ptr);
    } else if (iter->type() == dummy_velocity.type()) {
      auto var =
          dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(*iter);
      fuse_core::Variable::SharedPtr var_ptr =
          fuse_variables::VelocityLinear3DStamped::make_shared(var);
      graph.addVariable(var_ptr);
    } else if (iter->type() == dummy_imu_bias_gyro.type()) {
      auto var =
          dynamic_cast<const beam_variables::ImuBiasGyro3DStamped&>(*iter);
      fuse_core::Variable::SharedPtr var_ptr =
          beam_variables::ImuBiasGyro3DStamped::make_shared(var);
      graph.addVariable(var_ptr);
    } else if (iter->type() == dummy_imu_bias_accel.type()) {
      auto var =
          dynamic_cast<const beam_variables::ImuBiasAccel3DStamped&>(*iter);
      fuse_core::Variable::SharedPtr var_ptr =
          beam_variables::ImuBiasAccel3DStamped::make_shared(var);
      graph.addVariable(var_ptr);
    } else {
      return uuids;
    }
    uuids.push_back(iter->uuid());
  }
  return uuids;
}

TEST(ImuPreintegration, BaseFunctionality) {
  // set intrinsic noise of imu to zero
  ImuPreintegration::Params params;
  params.cov_gyro_noise.setZero();
  params.cov_accel_noise.setZero();
  params.cov_gyro_bias.setZero();
  params.cov_accel_bias.setZero();

  // set gravitional acceleration according to the basalt library
  params.gravitational_acceleration = GRAVITY;

  // instantiate preintegration class with zero noise. By default,
  // bias terms (i.e. bg, ba) are set to zero
  ImuPreintegration imu_preintegration = ImuPreintegration(params);

  // generate random ground truth trajectory of imu using splines
  int num_knots = 15;              // number of knots in spline curve
  double start_time_ns = 0;        // set start time to 0 seconds
  double time_interval_ns = 10e9;  // 10 seconds btw each knot
  double time_duration = 20e9;     // relative duration of simulation from start
  double time_simulation_ns =
      start_time_ns + time_duration;  // absolute time of simulation

  // generate spline curve of second order.
  basalt::Se3Spline<5> gt_spline(time_interval_ns, start_time_ns);
  gt_spline.genRandomTrajectory(num_knots);

  // from ground truth spline curve, get pose and derivative relationships
  // necessary to create synthetic imu measurements with zero noise and zero
  // bias
  double dt_ns = 1e7;  // assume data at 100 Hz
  static const Eigen::Vector3d gravity(0, 0, -GRAVITY);
  for (double t_ns = start_time_ns; t_ns < time_simulation_ns + dt_ns;
       t_ns += dt_ns) {
    // get rotational velocity and linear acceleration at sample rate
    Sophus::SE3d pose = gt_spline.pose(t_ns + dt_ns / 2);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns + dt_ns / 2);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns + dt_ns / 2) - gravity);

    // populate imu data type with synthetic readings
    ImuPreintegration::ImuData imu_data;
    imu_data.t_ros = ros::Time(t_ns / 1e9);
    imu_data.t = t_ns / 1e9;    // [sec]
    imu_data.w = rot_vel_body;  // [rad/sec]
    imu_data.a = accel_body;    // [m/sec^2]

    // populate buffers
    imu_preintegration.PopulateBuffer(imu_data);
  }

  // get pose and velocity of imu wrt world at start of simulation
  Sophus::SE3d start_pose = gt_spline.pose(start_time_ns);
  Eigen::Vector3d start_velocity_vec = gt_spline.transVelWorld(start_time_ns);

  // set start time and state of imu preintegration
  ros::Time start_time = ros::Time(start_time_ns / 1e9);
  Eigen::Matrix3d start_orientation_mat = start_pose.so3().matrix();
  Eigen::Vector3d start_position_vec = start_pose.translation();
  Eigen::Quaterniond start_orientation_quat(start_orientation_mat);

  // set start of imu preintegration. This requires us to pass three fuse
  // variables, which for testing purposes will match the ground truth of
  // the spline at the start of the simulation
  fuse_core::UUID device_id = fuse_core::uuid::generate(params.source);

  fuse_variables::Orientation3DStamped::SharedPtr start_orientation =
      fuse_variables::Orientation3DStamped::make_shared(start_time, device_id);
  start_orientation->w() = start_orientation_quat.w();
  start_orientation->x() = start_orientation_quat.x();
  start_orientation->y() = start_orientation_quat.y();
  start_orientation->z() = start_orientation_quat.z();

  fuse_variables::Position3DStamped::SharedPtr start_position =
      fuse_variables::Position3DStamped::make_shared(start_time, device_id);
  start_position->x() = start_position_vec[0];
  start_position->y() = start_position_vec[1];
  start_position->z() = start_position_vec[2];

  fuse_variables::VelocityLinear3DStamped::SharedPtr start_velocity =
      fuse_variables::VelocityLinear3DStamped::make_shared(start_time,
                                                           device_id);
  start_velocity->x() = start_velocity_vec[0];
  start_velocity->y() = start_velocity_vec[1];
  start_velocity->z() = start_velocity_vec[2];

  imu_preintegration.SetStart(start_time, start_orientation, start_position,
                              start_velocity);
  ImuState start_imu_state = imu_preintegration.GetImuState();

  // check
  EXPECT_EQ(start_imu_state.Stamp(), start_time);
  EXPECT_EQ(start_imu_state.Orientation().data()[0],
            start_orientation->data()[0]);
  EXPECT_EQ(start_imu_state.Orientation().data()[1],
            start_orientation->data()[1]);
  EXPECT_EQ(start_imu_state.Orientation().data()[2],
            start_orientation->data()[2]);
  EXPECT_EQ(start_imu_state.Orientation().data()[3],
            start_orientation->data()[3]);
  EXPECT_EQ(start_imu_state.Position().data()[0], start_position->data()[0]);
  EXPECT_EQ(start_imu_state.Position().data()[1], start_position->data()[1]);
  EXPECT_EQ(start_imu_state.Position().data()[2], start_position->data()[2]);
  EXPECT_EQ(start_imu_state.Velocity().data()[0], start_velocity->data()[0]);
  EXPECT_EQ(start_imu_state.Velocity().data()[1], start_velocity->data()[1]);
  EXPECT_EQ(start_imu_state.Velocity().data()[2], start_velocity->data()[2]);
  EXPECT_EQ(start_imu_state.BiasGyroscopeVec(), Eigen::Vector3d::Zero());
  EXPECT_EQ(start_imu_state.BiasAccelerationVec(), Eigen::Vector3d::Zero());

  // get pose and velocity of imu wrt world at end of simulation
  Sophus::SE3d end_pose = gt_spline.pose(time_simulation_ns);
  Eigen::Vector3d end_velocity_vec =
      gt_spline.transVelWorld(time_simulation_ns);

  // set end time and state of imu preintegration
  ros::Time end_time = ros::Time(time_simulation_ns / 1e9);
  Eigen::Matrix3d end_orientation_mat = end_pose.so3().matrix();
  Eigen::Vector3d end_position_vec = end_pose.translation();
  Eigen::Quaterniond end_orientation_quat(end_orientation_mat);

  // set end of imu preintegration. This requires us to pass two fuse
  // variables, which for testing purposes will match the ground truth of
  // the spline curve at the end of the simulation
  fuse_variables::Orientation3DStamped::SharedPtr end_orientation =
      fuse_variables::Orientation3DStamped::make_shared(end_time, device_id);
  end_orientation->w() = end_orientation_quat.w();
  end_orientation->x() = end_orientation_quat.x();
  end_orientation->y() = end_orientation_quat.y();
  end_orientation->z() = end_orientation_quat.z();

  fuse_variables::Position3DStamped::SharedPtr end_position =
      fuse_variables::Position3DStamped::make_shared(end_time, device_id);
  end_position->x() = end_position_vec[0];
  end_position->y() = end_position_vec[1];
  end_position->z() = end_position_vec[2];

  // calculate relative change-in-motion ground truth
  double delta_t = ros::Duration(end_time - start_time).toSec();
  Eigen::Matrix3d R_i = start_orientation_quat.toRotationMatrix();
  Eigen::Matrix3d delta_R =
      R_i.transpose() * end_orientation_quat.toRotationMatrix();
  Eigen::Vector3d delta_V =
      R_i.transpose() *
      (end_velocity_vec - start_velocity_vec - gravity * delta_t);
  Eigen::Vector3d delta_P =
      R_i.transpose() *
      (end_position_vec - start_position_vec - start_velocity_vec * delta_t -
       0.5 * gravity * delta_t * delta_t);

  // populate Preintegrator class from Slamtools
  PreIntegrator pre_integrator_gt;
  pre_integrator_gt.delta.t = delta_t;
  pre_integrator_gt.delta.q = delta_R;
  pre_integrator_gt.delta.p = delta_P;
  pre_integrator_gt.delta.v = delta_V;

  // predict end state using relative change-in-motion ground truth
  ImuState end_imu_state_pred =
      imu_preintegration.PredictState(pre_integrator_gt, start_imu_state);

  // check
  EXPECT_EQ(end_imu_state_pred.Stamp(), end_time);
  EXPECT_NEAR(end_imu_state_pred.OrientationQuat().w(),
              end_orientation_quat.w(), 1e-6);
  EXPECT_NEAR(end_imu_state_pred.OrientationQuat().x(),
              end_orientation_quat.x(), 1e-6);
  EXPECT_NEAR(end_imu_state_pred.OrientationQuat().y(),
              end_orientation_quat.y(), 1e-6);
  EXPECT_NEAR(end_imu_state_pred.OrientationQuat().z(),
              end_orientation_quat.z(), 1e-6);
  EXPECT_NEAR(end_imu_state_pred.PositionVec()[0], end_position_vec[0], 1e-6);
  EXPECT_NEAR(end_imu_state_pred.PositionVec()[1], end_position_vec[1], 1e-6);
  EXPECT_NEAR(end_imu_state_pred.PositionVec()[2], end_position_vec[2], 1e-6);
  EXPECT_NEAR(end_imu_state_pred.VelocityVec()[0], end_velocity_vec[0], 1e-6);
  EXPECT_NEAR(end_imu_state_pred.VelocityVec()[1], end_velocity_vec[1], 1e-6);
  EXPECT_NEAR(end_imu_state_pred.VelocityVec()[2], end_velocity_vec[2], 1e-6);

  // generate transaction to perform imu preintegration
  auto transaction =
      imu_preintegration.RegisterNewImuPreintegratedFactor(end_time);

  // get end imu state from preintegration
  ImuState end_imu_state = imu_preintegration.GetImuState();

  // check
  EXPECT_EQ(end_imu_state.Stamp(), end_time);
  EXPECT_NEAR(end_imu_state.Orientation().data()[0], end_orientation->data()[0],
              1e-6);
  EXPECT_NEAR(end_imu_state.Orientation().data()[1], end_orientation->data()[1],
              1e-6);
  EXPECT_NEAR(end_imu_state.Orientation().data()[2], end_orientation->data()[2],
              1e-6);
  EXPECT_NEAR(end_imu_state.Orientation().data()[3], end_orientation->data()[3],
              1e-6);
  EXPECT_NEAR(end_imu_state.Position().data()[0], end_position->data()[0],
              1e-3);
  EXPECT_NEAR(end_imu_state.Position().data()[1], end_position->data()[1],
              1e-3);
  EXPECT_NEAR(end_imu_state.Position().data()[2], end_position->data()[2],
              1e-4);
  EXPECT_NEAR(end_imu_state.Velocity().data()[0], end_velocity_vec[0], 1e-3);
  EXPECT_NEAR(end_imu_state.Velocity().data()[1], end_velocity_vec[1], 1e-3);
  EXPECT_NEAR(end_imu_state.Velocity().data()[2], end_velocity_vec[2], 1e-4);
  EXPECT_EQ(end_imu_state.BiasGyroscopeVec(), Eigen::Vector3d::Zero());
  EXPECT_EQ(end_imu_state.BiasAccelerationVec(), Eigen::Vector3d::Zero());

  // Create the graph
  fuse_graphs::HashGraph graph;

  // validate stamps
  EXPECT_TRUE(transaction.GetTransaction()->stamp() == end_imu_state.Stamp());

  // add variables and validate uuids for each transaction
  std::vector<fuse_core::UUID> transaction_uuids;
  transaction_uuids = AddVariables(transaction.GetTransaction(), graph);
  EXPECT_TRUE(transaction_uuids.size() == 10);

  std::vector<fuse_core::UUID> state_uuids;
  state_uuids.emplace_back(start_imu_state.Orientation().uuid());
  state_uuids.emplace_back(start_imu_state.Position().uuid());
  state_uuids.emplace_back(start_imu_state.Velocity().uuid());
  state_uuids.emplace_back(start_imu_state.BiasGyroscope().uuid());
  state_uuids.emplace_back(start_imu_state.BiasAcceleration().uuid());
  state_uuids.emplace_back(end_imu_state.Orientation().uuid());
  state_uuids.emplace_back(end_imu_state.Position().uuid());
  state_uuids.emplace_back(end_imu_state.Velocity().uuid());
  state_uuids.emplace_back(end_imu_state.BiasGyroscope().uuid());
  state_uuids.emplace_back(end_imu_state.BiasAcceleration().uuid());

  std::sort(transaction_uuids.begin(), transaction_uuids.end());
  std::sort(state_uuids.begin(), state_uuids.end());
  EXPECT_TRUE(transaction_uuids == state_uuids);

  // // add constraints and validate for each transaction
  int counter{0};
  counter += AddConstraints(transaction.GetTransaction(), graph);
  EXPECT_TRUE(counter == 2);

  // Optimize the constraints and variables.
  graph.optimize();

  // auto o1 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
  //     graph.getVariable(start_imu_state.Orientation().uuid()));
  // auto o2 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
  //     graph.getVariable(end_imu_state.Orientation().uuid()));

  // auto p1 = dynamic_cast<const fuse_variables::Position3DStamped&>(
  //     graph.getVariable(start_imu_state.Position().uuid()));
  // auto p2 = dynamic_cast<const fuse_variables::Position3DStamped&>(
  //     graph.getVariable(end_imu_state.Position().uuid()));

  // auto v1 = dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(
  //     graph.getVariable(start_imu_state.Velocity().uuid()));
  // auto v2 = dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(
  //     graph.getVariable(end_imu_state.Velocity().uuid()));

  // auto bg1 = dynamic_cast<const beam_variables::ImuBiasGyro3DStamped&>(
  //     graph.getVariable(start_imu_state.BiasGyroscope().uuid()));
  // auto bg2 = dynamic_cast<const beam_variables::ImuBiasGyro3DStamped&>(
  //     graph.getVariable(end_imu_state.BiasGyroscope().uuid()));

  // auto ba1 = dynamic_cast<const beam_variables::ImuBiasAccel3DStamped&>(
  //     graph.getVariable(start_imu_state.BiasAcceleration().uuid()));
  // auto ba2 = dynamic_cast<const beam_variables::ImuBiasAccel3DStamped&>(
  //     graph.getVariable(end_imu_state.BiasAcceleration().uuid()));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
