#include <basalt/spline/se3_spline.h>
#include <gtest/gtest.h>
#include <fuse_core/constraint.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_graphs/hash_graph.h>

#include <bs_constraints/frame_to_frame/relative_constraint.h>
#include <bs_constraints/global/absolute_constraint.h>
#include <bs_models/frame_to_frame/imu_preintegration.h>
#include <bs_common/utils/utils_tests.h>

using namespace fuse_constraints;
using namespace bs_common;
using namespace bs_constraints::frame_to_frame;
using namespace bs_constraints::global;
using namespace bs_models::frame_to_frame;

void CalculateRelativeMotion(const ImuState& IS1, const ImuState& IS2,
                             ros::Duration& delta_t,
                             Eigen::Quaterniond& delta_q,
                             Eigen::Vector3d& delta_p,
                             Eigen::Vector3d& delta_v) {
  // calculate change in time
  delta_t = ros::Duration(IS2.Stamp() - IS1.Stamp());
  const double& dt = delta_t.toSec();

  // calculate change in motion
  const Eigen::Matrix3d& R1_transpose = IS1.OrientationMat().transpose();
  delta_q = R1_transpose * IS2.OrientationMat();
  delta_v = R1_transpose *
            (IS2.VelocityVec() - IS1.VelocityVec() - GRAVITY_WORLD * dt);
  delta_p =
      R1_transpose * (IS2.PositionVec() - IS1.PositionVec() -
                      IS1.VelocityVec() * dt - 0.5 * GRAVITY_WORLD * dt * dt);
}

class Data {
 public:
  Data() {
    // set time of simulation and gravity vector
    int64_t time_simulation_ns = start_time_ns + time_duration;

    // set times of imu states
    ros::Time t1_ros = ros::Time(start_time_ns * 1e-9);
    ros::Time t2_ros =
        ros::Time((start_time_ns + time_simulation_ns) * 0.5 * 1e-9);
    ros::Time t3_ros = ros::Time(time_simulation_ns * 1e-9);

    // generate spline
    basalt::Se3Spline<5> gt_spline(time_interval_ns, start_time_ns);
    gt_spline.genRandomTrajectory(num_knots);

    // create synthetic imu measurements
    for (int64_t t_ns = start_time_ns; t_ns < time_simulation_ns + dt_ns;
         t_ns += dt_ns) {
      // get state info in middle of interval
      Sophus::SE3d pose = gt_spline.pose(t_ns + dt_ns / 2);
      Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns + dt_ns / 2);
      Eigen::Vector3d lin_accel_body =
          pose.so3().inverse() *
          (gt_spline.transAccelWorld(t_ns + dt_ns / 2) - GRAVITY_WORLD);

      // assign info to start of interval in imu data
      bs_common::IMUData imu_data;
      imu_data.t = ros::Time(t_ns * 1e-9);  // [sec]
      imu_data.w = rot_vel_body;            // [rad/sec]
      imu_data.a = lin_accel_body;          // [m/sec^2]

      // populate ground truth imu measurements buffer
      imu_data_gt.emplace_back(imu_data);

      // get ground truth pose every second between start and end exclusive
      if (t_ns % int64_t(1e9) == 0 && t_ns > start_time_ns &&
          t_ns < time_simulation_ns) {
        Eigen::Matrix3d q_k_mat = gt_spline.pose(t_ns).so3().matrix();
        Eigen::Vector3d p_k_vec = gt_spline.pose(t_ns).translation();
        Eigen::Quaterniond q_k_quat(q_k_mat);
        Eigen::Matrix4d T_WORLD_IMU_k;
        beam::QuaternionAndTranslationToTransformMatrix(q_k_quat, p_k_vec,
                                                        T_WORLD_IMU_k);
        pose_gt.emplace_back(T_WORLD_IMU_k);
      }
    }

    // set Imu State 1
    Sophus::SE3d pose1 = gt_spline.pose(t1_ros.toNSec());
    Eigen::Matrix3d q1_mat = pose1.so3().matrix();
    q1_quat = q1_mat;
    p1_vec = pose1.translation();
    v1_vec = gt_spline.transVelWorld(t1_ros.toNSec());
    bs_common::ImuState IS1_temp(t1_ros, q1_quat, p1_vec, v1_vec);
    IS1 = std::move(IS1_temp);

    // set Imu State 2
    Sophus::SE3d pose2 = gt_spline.pose(t2_ros.toNSec());
    Eigen::Matrix3d q2_mat = pose2.so3().matrix();
    q2_quat = q2_mat;
    p2_vec = pose2.translation();
    v2_vec = gt_spline.transVelWorld(t2_ros.toNSec());
    bs_common::ImuState IS2_temp(t2_ros, q2_quat, p2_vec, v2_vec);
    IS2 = std::move(IS2_temp);

    // set Imu State 3
    Sophus::SE3d pose3 = gt_spline.pose(t3_ros.toNSec());
    Eigen::Matrix3d q3_mat = pose3.so3().matrix();
    q3_quat = q3_mat;
    p3_vec = pose3.translation();
    v3_vec = gt_spline.transVelWorld(t3_ros.toNSec());
    bs_common::ImuState IS3_temp(t3_ros, q3_quat, p3_vec, v3_vec);
    IS3 = std::move(IS3_temp);

    // calculate relative motion deltas between states
    CalculateRelativeMotion(IS1, IS2, delta_t_12, delta_q_12, delta_p_12,
                            delta_v_12);
    CalculateRelativeMotion(IS2, IS3, delta_t_23, delta_q_23, delta_p_23,
                            delta_v_23);
  }

  // spline parameters
  int num_knots = 15;
  int64_t start_time_ns = 0;        // [nano sec]
  int64_t time_interval_ns = 10e9;  // [nano sec]
  int64_t time_duration = 20e9;     // [nano sec]
  int64_t dt_ns = 1e7;              // [nano sec]

  // buffers
  std::vector<bs_common::IMUData> imu_data_gt;
  std::vector<Eigen::Matrix4d> pose_gt;

  // Imu State 1
  bs_common::ImuState IS1;
  Eigen::Quaterniond q1_quat;
  Eigen::Vector3d p1_vec;
  Eigen::Vector3d v1_vec;

  // Imu State 2
  bs_common::ImuState IS2;
  Eigen::Quaterniond q2_quat;
  Eigen::Vector3d p2_vec;
  Eigen::Vector3d v2_vec;

  // Imu State 3
  bs_common::ImuState IS3;
  Eigen::Quaterniond q3_quat;
  Eigen::Vector3d p3_vec;
  Eigen::Vector3d v3_vec;

  // Imu State deltas
  ros::Duration delta_t_12;
  Eigen::Quaterniond delta_q_12;
  Eigen::Vector3d delta_p_12;
  Eigen::Vector3d delta_v_12;

  ros::Duration delta_t_23;
  Eigen::Quaterniond delta_q_23;
  Eigen::Vector3d delta_p_23;
  Eigen::Vector3d delta_v_23;
};

int AddConstraints(const fuse_core::Transaction::SharedPtr& transaction,
                   fuse_graphs::HashGraph& graph) {
  // instantiate constraints counter
  int counter{0};

  // instantiate dummy constraints
  bs_constraints::frame_to_frame::RelativeImuState3DStampedConstraint
      dummy_relative_constraint;
  bs_constraints::global::AbsoluteImuState3DStampedConstraint
      dummy_absolute_constraint;

  // add constraints from transaction to hashgraph
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
  // instantiate dummy fuse/beam variables
  fuse_variables::Orientation3DStamped dummy_orientation;
  fuse_variables::Position3DStamped dummy_position;
  fuse_variables::VelocityLinear3DStamped dummy_velocity;
  bs_variables::GyroscopeBias3DStamped dummy_imu_bias_gyro;
  bs_variables::AccelerationBias3DStamped dummy_imu_bias_accel;
  std::vector<fuse_core::UUID> uuids;

  // add variables from transaction to hashgraph
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
          dynamic_cast<const bs_variables::GyroscopeBias3DStamped&>(*iter);
      fuse_core::Variable::SharedPtr var_ptr =
          bs_variables::GyroscopeBias3DStamped::make_shared(var);
      graph.addVariable(var_ptr);
    } else if (iter->type() == dummy_imu_bias_accel.type()) {
      auto var =
          dynamic_cast<const bs_variables::AccelerationBias3DStamped&>(*iter);
      fuse_core::Variable::SharedPtr var_ptr =
          bs_variables::AccelerationBias3DStamped::make_shared(var);
      graph.addVariable(var_ptr);
    } else {
      return uuids;
    }
    uuids.push_back(iter->uuid());
  }
  return uuids;
}

TEST(ImuPreintegration, Simple2StateFG) {
  // create arbitrary state values
  Eigen::Quaterniond q1_quat(0.952, 0.038, -0.189, 0.239);
  Eigen::Vector3d p1_vec(1.5, -3.0, 1.0);
  Eigen::Vector3d v1_vec(1.5, -3.0, 1.0);
  Eigen::Vector3d bg1_vec(4e-5, 5e-5, 6e-5);
  Eigen::Vector3d ba1_vec(1e-5, 2e-5, 3e-5);

  Eigen::Quaterniond q2_quat(0.944, -0.128, 0.145, -0.269);
  Eigen::Vector3d p2_vec(-1.5, 3.0, -1.0);
  Eigen::Vector3d v2_vec(-1.5, 3.0, -1.0);
  Eigen::Vector3d bg2_vec(4e-5, 5e-5, 6e-5);
  Eigen::Vector3d ba2_vec(1e-5, 2e-5, 3e-5);

  // instantiate IMU states
  bs_common::ImuState IS1(ros::Time(1), q1_quat, p1_vec, v1_vec, bg1_vec,
                          ba1_vec);
  bs_common::ImuState IS2(ros::Time(2), q2_quat, p2_vec, v2_vec, bg2_vec,
                          ba2_vec);

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Add variables
  fuse_variables::Orientation3DStamped::SharedPtr o1 =
      fuse_variables::Orientation3DStamped::make_shared(IS1.Orientation());
  fuse_variables::Position3DStamped::SharedPtr p1 =
      fuse_variables::Position3DStamped::make_shared(IS1.Position());
  fuse_variables::VelocityLinear3DStamped::SharedPtr v1 =
      fuse_variables::VelocityLinear3DStamped::make_shared(IS1.Velocity());
  bs_variables::GyroscopeBias3DStamped::SharedPtr bg1 =
      bs_variables::GyroscopeBias3DStamped::make_shared(IS1.GyroBias());
  bs_variables::AccelerationBias3DStamped::SharedPtr ba1 =
      bs_variables::AccelerationBias3DStamped::make_shared(IS1.AccelBias());

  fuse_variables::Orientation3DStamped::SharedPtr o2 =
      fuse_variables::Orientation3DStamped::make_shared(IS2.Orientation());
  fuse_variables::Position3DStamped::SharedPtr p2 =
      fuse_variables::Position3DStamped::make_shared(IS2.Position());
  fuse_variables::VelocityLinear3DStamped::SharedPtr v2 =
      fuse_variables::VelocityLinear3DStamped::make_shared(IS2.Velocity());
  bs_variables::GyroscopeBias3DStamped::SharedPtr bg2 =
      bs_variables::GyroscopeBias3DStamped::make_shared(IS2.GyroBias());
  bs_variables::AccelerationBias3DStamped::SharedPtr ba2 =
      bs_variables::AccelerationBias3DStamped::make_shared(IS2.AccelBias());

  graph.addVariable(o1);
  graph.addVariable(p1);
  graph.addVariable(v1);
  graph.addVariable(bg1);
  graph.addVariable(ba1);

  graph.addVariable(o2);
  graph.addVariable(p2);
  graph.addVariable(v2);
  graph.addVariable(bg2);
  graph.addVariable(ba2);

  // Create an absolute pose constraint at the origin
  fuse_core::Vector7d pose_mean_origin;
  pose_mean_origin << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  fuse_core::Matrix6d pose_cov_origin = fuse_core::Matrix6d::Identity();
  auto prior_pose = AbsolutePose3DStampedConstraint::make_shared(
      "test", *p1, *o1, pose_mean_origin, pose_cov_origin);

  // Create an absolute linear velocity constraint at the origin
  fuse_core::Vector3d vel_mean_origin;
  vel_mean_origin << 0.0, 0.0, 0.0;
  fuse_core::Matrix3d vel_cov_origin = fuse_core::Matrix3d::Identity();
  auto prior_vel = AbsoluteVelocityLinear3DStampedConstraint::make_shared(
      "test", *v1, vel_mean_origin, vel_cov_origin);

  // Create absolute bias constraint at zero
  fuse_core::Vector3d bias_mean_origin;
  bias_mean_origin << 0.0, 0.0, 0.0;
  fuse_core::Matrix3d bias_cov_origin = fuse_core::Matrix3d::Identity();
  auto prior_bg = AbsoluteGyroBias3DStampedConstraint::make_shared(
      "test", *bg1, bias_mean_origin, bias_cov_origin);
  auto prior_ba = AbsoluteAccelBias3DStampedConstraint::make_shared(
      "test", *ba1, bias_mean_origin, bias_cov_origin);

  // Create a relative pose constraint for 1m in the x direction
  fuse_core::Vector7d pose_mean_delta;
  pose_mean_delta << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  fuse_core::Matrix6d pose_cov_delta = fuse_core::Matrix6d::Identity();
  auto relative_pose = RelativePose3DStampedConstraint::make_shared(
      "test", *p1, *o1, *p2, *o2, pose_mean_delta, pose_cov_delta);

  // Create a relative linear velocity constraint for 1m/s in the x direction
  fuse_core::Vector3d vel_mean_delta;
  vel_mean_delta << 1.0, 0.0, 0.0;
  fuse_core::Matrix3d vel_cov_delta = fuse_core::Matrix3d::Identity();
  auto relative_vel = RelativeVelocityLinear3DStampedConstraint::make_shared(
      "test", *v1, *v2, vel_mean_delta, vel_cov_delta);

  // Create relative bias constraints for 0.001 in the x direction
  fuse_core::Vector3d bias_mean_delta;
  bias_mean_delta << 0.001, 0.0, 0.0;
  fuse_core::Matrix3d bias_cov_delta = fuse_core::Matrix3d::Identity();
  auto relative_bg = RelativeGyroBias3DStampedConstraint::make_shared(
      "test", *bg1, *bg2, bias_mean_delta, bias_cov_delta);
  auto relative_ba = RelativeAccelBias3DStampedConstraint::make_shared(
      "test", *ba1, *ba2, bias_mean_delta, bias_cov_delta);

  // get means
  Eigen::Matrix<double, 16, 1> mean_origin;
  Eigen::Matrix<double, 16, 1> mean_delta;
  mean_origin << pose_mean_origin, vel_mean_origin, bias_mean_origin,
      bias_mean_delta;
  mean_delta << pose_mean_delta, vel_mean_delta, bias_mean_delta,
      bias_mean_delta;

  // Add constraints
  graph.addConstraint(prior_pose);
  graph.addConstraint(prior_vel);
  graph.addConstraint(prior_bg);
  graph.addConstraint(prior_ba);

  graph.addConstraint(relative_pose);
  graph.addConstraint(relative_vel);
  graph.addConstraint(relative_bg);
  graph.addConstraint(relative_ba);

  // Optimize
  graph.optimize();

  // Update IMU states with optimized graph
  auto g = fuse_graphs::HashGraph::make_shared(graph);
  IS1.Update(g);
  IS2.Update(g);

  // Check
  EXPECT_EQ(1, IS1.Updates());
  EXPECT_NEAR(pose_mean_origin[3], IS1.Orientation().w(), 1.0e-3);
  EXPECT_NEAR(pose_mean_origin[4], IS1.Orientation().x(), 1.0e-3);
  EXPECT_NEAR(pose_mean_origin[5], IS1.Orientation().y(), 1.0e-3);
  EXPECT_NEAR(pose_mean_origin[6], IS1.Orientation().z(), 1.0e-3);
  EXPECT_NEAR(pose_mean_origin[0], IS1.Position().x(), 1.0e-5);
  EXPECT_NEAR(pose_mean_origin[1], IS1.Position().y(), 1.0e-5);
  EXPECT_NEAR(pose_mean_origin[2], IS1.Position().z(), 1.0e-5);
  EXPECT_NEAR(vel_mean_origin[0], IS1.Velocity().x(), 1.0e-5);
  EXPECT_NEAR(vel_mean_origin[1], IS1.Velocity().y(), 1.0e-5);
  EXPECT_NEAR(vel_mean_origin[2], IS1.Velocity().z(), 1.0e-5);
  EXPECT_NEAR(bias_mean_origin[0], IS1.GyroBias().x(), 1.0e-5);
  EXPECT_NEAR(bias_mean_origin[1], IS1.GyroBias().y(), 1.0e-5);
  EXPECT_NEAR(bias_mean_origin[2], IS1.GyroBias().z(), 1.0e-5);
  EXPECT_NEAR(bias_mean_origin[0], IS1.AccelBias().x(), 1.0e-5);
  EXPECT_NEAR(bias_mean_origin[1], IS1.AccelBias().y(), 1.0e-5);
  EXPECT_NEAR(bias_mean_origin[2], IS1.AccelBias().z(), 1.0e-5);

  EXPECT_EQ(1, IS2.Updates());
  EXPECT_NEAR(pose_mean_delta[3], IS2.Orientation().w(), 1.0e-3);
  EXPECT_NEAR(pose_mean_delta[4], IS2.Orientation().x(), 1.0e-3);
  EXPECT_NEAR(pose_mean_delta[5], IS2.Orientation().y(), 1.0e-3);
  EXPECT_NEAR(pose_mean_delta[6], IS2.Orientation().z(), 1.0e-3);
  EXPECT_NEAR(pose_mean_delta[0], IS2.Position().x(), 1.0e-5);
  EXPECT_NEAR(pose_mean_delta[1], IS2.Position().y(), 1.0e-5);
  EXPECT_NEAR(pose_mean_delta[2], IS2.Position().z(), 1.0e-5);
  EXPECT_NEAR(vel_mean_delta[0], IS2.Velocity().x(), 1.0e-5);
  EXPECT_NEAR(vel_mean_delta[1], IS2.Velocity().y(), 1.0e-5);
  EXPECT_NEAR(vel_mean_delta[2], IS2.Velocity().z(), 1.0e-5);
  EXPECT_NEAR(bias_mean_delta[0], IS2.GyroBias().x(), 1.0e-5);
  EXPECT_NEAR(bias_mean_delta[1], IS2.GyroBias().y(), 1.0e-5);
  EXPECT_NEAR(bias_mean_delta[2], IS2.GyroBias().z(), 1.0e-5);
  EXPECT_NEAR(bias_mean_delta[0], IS2.AccelBias().x(), 1.0e-5);
  EXPECT_NEAR(bias_mean_delta[1], IS2.AccelBias().y(), 1.0e-5);
  EXPECT_NEAR(bias_mean_delta[2], IS2.AccelBias().z(), 1.0e-5);
}

class ImuPreintegration_ZeroNoiseZeroBias : public ::testing::Test {
 public:
  void SetUp() override {
    // set intrinsic noise of imu to zero
    params.cov_prior_noise = 1e9;
    params.cov_gyro_noise.setZero();
    params.cov_accel_noise.setZero();
    params.cov_gyro_bias.setZero();
    params.cov_accel_bias.setZero();

    // instantiate preintegration class with zero noise. By default,
    // bias terms (i.e. bg, ba) are set to zero
    imu_preintegration = std::make_unique<ImuPreintegration>(params);

    // populate ImuPreintegration with synthetic imu measurements
    for (bs_common::IMUData msg : data.imu_data_gt)
      imu_preintegration->AddToBuffer(msg);

    // get copies of imu states
    IS1 = data.IS1;
    IS2 = data.IS2;
    IS3 = data.IS3;

    // get stamps of imu states
    t_start = IS1.Stamp();
    t_middle = IS2.Stamp();
    t_end = IS3.Stamp();
  }

  Data data;
  ImuPreintegration::Params params;
  std::unique_ptr<ImuPreintegration> imu_preintegration;

  ImuState IS1;
  ImuState IS2;
  ImuState IS3;

  ros::Time t_start;
  ros::Time t_middle;
  ros::Time t_end;
};

TEST_F(ImuPreintegration_ZeroNoiseZeroBias, BaseFunctionality) {
  /**
   * CheckParameters() functionality
   */

  // instantiate preintegration class with invalid prior noise
  EXPECT_ANY_THROW({
    ImuPreintegration::Params params;
    params.cov_prior_noise = 0;
    std::unique_ptr<ImuPreintegration> dummy_imu_preintegration =
        std::make_unique<ImuPreintegration>(params);
  });

  /**
   * SetStart() functionality
   */

  // set start of imu preintegration. This requires us to pass three fuse
  // variables, which for testing purposes will match IS1
  fuse_variables::Orientation3DStamped::SharedPtr o_start =
      fuse_variables::Orientation3DStamped::make_shared(IS1.Orientation());
  fuse_variables::Position3DStamped::SharedPtr p_start =
      fuse_variables::Position3DStamped::make_shared(IS1.Position());
  fuse_variables::VelocityLinear3DStamped::SharedPtr v_start =
      fuse_variables::VelocityLinear3DStamped::make_shared(IS1.Velocity());

  // check default
  imu_preintegration->SetStart(t_start);
  bs_common::ImuState IS_default(t_start);
  bs_common::ImuState IS_start_default = imu_preintegration->GetImuState();
  ExpectImuStateEq(IS_start_default, IS_default);

  // check optional initialization
  imu_preintegration->SetStart(t_start, o_start, p_start, v_start);
  bs_common::ImuState IS_start = imu_preintegration->GetImuState();
  ExpectImuStateEq(IS_start, IS1);

  /**
   * PredictState() functionality
   */

  // populate Preintegrator class with imu preintegration deltas
  // from data class
  bs_common::PreIntegrator pre_integrator_12;
  pre_integrator_12.delta.t = data.delta_t_12;
  pre_integrator_12.delta.q = data.delta_q_12;
  pre_integrator_12.delta.p = data.delta_p_12;
  pre_integrator_12.delta.v = data.delta_v_12;

  bs_common::PreIntegrator pre_integrator_23;
  pre_integrator_23.delta.t = data.delta_t_23;
  pre_integrator_23.delta.q = data.delta_q_23;
  pre_integrator_23.delta.p = data.delta_p_23;
  pre_integrator_23.delta.v = data.delta_v_23;

  // predict middle and end imu state using relative change-in-motion ground
  // truth
  bs_common::ImuState IS_middle_predict =
      imu_preintegration->PredictState(pre_integrator_12, IS_start);
  bs_common::ImuState IS_end_predict =
      imu_preintegration->PredictState(pre_integrator_23, IS_middle_predict);

  // check
  ExpectImuStateEq(IS_middle_predict, IS2);
  ExpectImuStateEq(IS_end_predict, IS3);

  /**
   * GetPose() functionality
   */

  for (int i = 1; i - 1 < data.pose_gt.size(); i++) {
    Eigen::Matrix4d T_WORLD_IMU;
    imu_preintegration->GetPose(T_WORLD_IMU, ros::Time(i));
    ExpectTransformsNear(T_WORLD_IMU, data.pose_gt[i - 1]);
  }

  // expect false from incorrect time
  Eigen::Matrix4d T_WORLD_IMU;
  EXPECT_FALSE(imu_preintegration->GetPose(T_WORLD_IMU, t_start));

  /**
   * RegisterNewImuPreintegratedFactor() functionality
   */

  // expect false as incorrect time will return nullptr
  EXPECT_FALSE(imu_preintegration->RegisterNewImuPreintegratedFactor(t_start));

  // generate transaction to perform imu preintegration
  auto transaction =
      imu_preintegration->RegisterNewImuPreintegratedFactor(t_end);

  // get end imu state from preintegration
  bs_common::ImuState IS_end = imu_preintegration->GetImuState();

  // check
  ExpectImuStateNear(IS_end, IS3);

  // validate stamps
  EXPECT_TRUE(transaction->stamp() == IS_end.Stamp());

  // Create the graph
  fuse_graphs::HashGraph graph;

  // add variables and validate uuids for each transaction
  std::vector<fuse_core::UUID> transaction_uuids;
  transaction_uuids = AddVariables(transaction, graph);
  EXPECT_TRUE(transaction_uuids.size() == 10);

  std::vector<fuse_core::UUID> state_uuids;
  state_uuids.emplace_back(IS_start.Orientation().uuid());
  state_uuids.emplace_back(IS_start.Position().uuid());
  state_uuids.emplace_back(IS_start.Velocity().uuid());
  state_uuids.emplace_back(IS_start.GyroBias().uuid());
  state_uuids.emplace_back(IS_start.AccelBias().uuid());
  state_uuids.emplace_back(IS_end.Orientation().uuid());
  state_uuids.emplace_back(IS_end.Position().uuid());
  state_uuids.emplace_back(IS_end.Velocity().uuid());
  state_uuids.emplace_back(IS_end.GyroBias().uuid());
  state_uuids.emplace_back(IS_end.AccelBias().uuid());

  std::sort(transaction_uuids.begin(), transaction_uuids.end());
  std::sort(state_uuids.begin(), state_uuids.end());
  EXPECT_TRUE(transaction_uuids == state_uuids);

  // add constraints and validate for transaction
  int counter{0};
  counter += AddConstraints(transaction, graph);
  EXPECT_TRUE(counter == 2);

  // optimize the constraints and variables.
  graph.optimize();

  // update IMU states with optimized graph
  auto g = fuse_graphs::HashGraph::make_shared(graph);
  IS_start.Update(g);
  IS_end.Update(g);

  // check
  ExpectImuStateNear(IS1, IS_start);
  ExpectImuStateNear(IS3, IS_end);
}

TEST_F(ImuPreintegration_ZeroNoiseZeroBias, MultipleTransactions) {
  // set start
  fuse_variables::Orientation3DStamped::SharedPtr o_start =
      fuse_variables::Orientation3DStamped::make_shared(IS1.Orientation());
  fuse_variables::Position3DStamped::SharedPtr p_start =
      fuse_variables::Position3DStamped::make_shared(IS1.Position());
  fuse_variables::VelocityLinear3DStamped::SharedPtr v_start =
      fuse_variables::VelocityLinear3DStamped::make_shared(IS1.Velocity());
  imu_preintegration->SetStart(t_start, o_start, p_start, v_start);
  bs_common::ImuState IS_start = imu_preintegration->GetImuState();

  // for testing, call GetPose() from start to middle
  for (int i = t_start.toSec() + 1; i - 1 < t_middle.toSec(); i++) {
    Eigen::Matrix4d dummy_T_WORLD_IMU;
    imu_preintegration->GetPose(dummy_T_WORLD_IMU, ros::Time(i));
  }

  // generate transactions, taking start, middle, and end as key frames
  auto transaction1 =
      imu_preintegration->RegisterNewImuPreintegratedFactor(t_middle);
  bs_common::ImuState IS_middle = imu_preintegration->GetImuState();

  auto transaction2 =
      imu_preintegration->RegisterNewImuPreintegratedFactor(t_end);
  bs_common::ImuState IS_end = imu_preintegration->GetImuState();

  // create graph
  fuse_graphs::HashGraph graph;

  // add transactions
  graph.update(*transaction1);
  graph.optimize();

  graph.update(*transaction2);
  graph.optimize();

  // update IMU states with optimized graph
  auto g = fuse_graphs::HashGraph::make_shared(graph);
  IS_start.Update(g);
  IS_middle.Update(g);
  IS_end.Update(g);

  IS_start.Update(g);
  IS_end.Update(g);

  // check
  ExpectImuStateNear(IS1, IS_start);
  ExpectImuStateNear(IS2, IS_middle);
  ExpectImuStateNear(IS3, IS_end);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
