#include <basalt/spline/se3_spline.h>
#include <gtest/gtest.h>
#include <fuse_core/constraint.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_graphs/hash_graph.h>

#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <beam_utils/math.h>

using namespace beam_constraints::frame_to_frame;
using namespace beam_constraints::global;
using namespace beam_models::frame_to_frame;

void CalculateRelativeMotion(const ImuState& IS1, const ImuState& IS2,
                             Eigen::Quaterniond& delta_q,
                             Eigen::Vector3d& delta_p, Eigen::Vector3d& delta_v,
                             const Eigen::Vector3d& gravity,
                             bool imu_preintegration = true) {
  double dt = ros::Duration(IS2.Stamp() - IS1.Stamp()).toSec();
  Eigen::Vector3d g{gravity};

  if (!imu_preintegration) {
    dt = 0;
    g.setZero();
  }

  Eigen::Matrix3d q1_rot_trans =
      IS1.OrientationQuat().toRotationMatrix().transpose();
  delta_q = q1_rot_trans * IS2.OrientationQuat().toRotationMatrix();
  delta_v = q1_rot_trans * (IS2.VelocityVec() - IS1.VelocityVec() - g * dt);
  delta_p = q1_rot_trans * (IS2.PositionVec() - IS1.PositionVec() -
                            IS1.VelocityVec() * dt - 0.5 * g * dt * dt);
}

class Data {
 public:
  Data() {
    // set time of simulation and gravity vector
    int64_t time_simulation_ns = start_time_ns + time_duration;
    gravity << 0, 0, -GRAVITY;

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
          (gt_spline.transAccelWorld(t_ns + dt_ns / 2) - gravity);

      // assign info to start of interval in imu data
      beam_common::IMUData imu_data;
      imu_data.t = ros::Time(t_ns * 1e-9);  // [sec]
      imu_data.w = rot_vel_body;            // [rad/sec]
      imu_data.a = lin_accel_body;          // [m/sec^2]

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
    ImuState IS1_temp(t1_ros, q1_quat, p1_vec, v1_vec);
    IS1 = std::move(IS1_temp);

    // set Imu State 2
    Sophus::SE3d pose2 = gt_spline.pose(t2_ros.toNSec());
    Eigen::Matrix3d q2_mat = pose2.so3().matrix();
    q2_quat = q2_mat;
    p2_vec = pose2.translation();
    v2_vec = gt_spline.transVelWorld(t2_ros.toNSec());
    ImuState IS2_temp(t2_ros, q2_quat, p2_vec, v2_vec);
    IS2 = std::move(IS2_temp);

    // set Imu State 3
    Sophus::SE3d pose3 = gt_spline.pose(t3_ros.toNSec());
    Eigen::Matrix3d q3_mat = pose3.so3().matrix();
    q3_quat = q3_mat;
    p3_vec = pose3.translation();
    v3_vec = gt_spline.transVelWorld(t3_ros.toNSec());
    ImuState IS3_temp(t3_ros, q3_quat, p3_vec, v3_vec);
    IS3 = std::move(IS3_temp);

    // calculate relative motion deltas between states
    CalculateRelativeMotion(IS1, IS2, delta_q_12, delta_p_12, delta_v_12,
                            gravity);
    CalculateRelativeMotion(IS2, IS3, delta_q_23, delta_p_23, delta_v_23,
                            gravity);

    delta_t_12 = ros::Duration(IS2.Stamp() - IS1.Stamp());
    delta_t_23 = ros::Duration(IS3.Stamp() - IS2.Stamp());
  }

  // spline parameters
  int num_knots = 15;
  int64_t start_time_ns = 0;        // [nano sec]
  int64_t time_interval_ns = 10e9;  // [nano sec]
  int64_t time_duration = 20e9;     // [nano sec]
  int64_t dt_ns = 1e7;              // [nano sec]

  Eigen::Vector3d gravity;
  std::vector<beam_common::IMUData> imu_data_gt;
  std::vector<Eigen::Matrix4d> pose_gt;

  // Imu State 1
  ImuState IS1;
  Eigen::Quaterniond q1_quat;
  Eigen::Vector3d p1_vec;
  Eigen::Vector3d v1_vec;

  // Imu State 2
  ImuState IS2;
  Eigen::Quaterniond q2_quat;
  Eigen::Vector3d p2_vec;
  Eigen::Vector3d v2_vec;

  // Imu State 3
  ImuState IS3;
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

Eigen::Matrix<double, 16, 1> CalculateRelativeStateDelta(const ImuState& IS1,
                                                         const ImuState& IS2) {
  Eigen::Quaterniond delta_q;
  Eigen::Vector3d delta_p;
  Eigen::Vector3d delta_v;
  Eigen::Vector3d dummy_gravity{Eigen::Vector3d::Zero()};
  CalculateRelativeMotion(IS1, IS2, delta_q, delta_p, delta_v, dummy_gravity,
                          false);

  Eigen::Vector3d delta_bg = IS2.GyroBiasVec() - IS1.GyroBiasVec();
  Eigen::Vector3d delta_ba = IS2.AccelBiasVec() - IS1.AccelBiasVec();

  Eigen::Matrix<double, 16, 1> delta;
  delta << delta_q.w(), delta_q.vec(), delta_p, delta_v, delta_bg, delta_ba;
  return delta;
}

RelativeImuState3DStampedConstraint::SharedPtr CreateRelativeConstraint(
    const fuse_variables::Orientation3DStamped& orientation1,
    const fuse_variables::Position3DStamped& position1,
    const fuse_variables::VelocityLinear3DStamped& velocity1,
    const beam_variables::GyroscopeBias3DStamped& gyrobias1,
    const beam_variables::AccelerationBias3DStamped& accelbias1,
    const fuse_variables::Orientation3DStamped& orientation2,
    const fuse_variables::Position3DStamped& position2,
    const fuse_variables::VelocityLinear3DStamped& velocity2,
    const beam_variables::GyroscopeBias3DStamped& gyrobias2,
    const beam_variables::AccelerationBias3DStamped& accelbias2,
    const Eigen::Matrix<double, 16, 1>& delta,
    const Eigen::Matrix<double, 15, 15>& covariance) {
  auto constraint = RelativeImuState3DStampedConstraint::make_shared(
      "SOURCE", orientation1, position1, velocity1, gyrobias1, accelbias1,
      orientation2, position2, velocity2, gyrobias2, accelbias2, delta,
      covariance);
  return constraint;
}

AbsoluteImuState3DStampedConstraint::SharedPtr CreatePriorConstraint(
    const fuse_variables::Orientation3DStamped& orientation,
    const fuse_variables::Position3DStamped& position,
    const fuse_variables::VelocityLinear3DStamped& velocity,
    const beam_variables::GyroscopeBias3DStamped& gyrobias,
    const beam_variables::AccelerationBias3DStamped& accelbias) {
  Eigen::Matrix<double, 16, 1> mean;
  mean << orientation.w(), orientation.x(), orientation.y(), orientation.z(),
      position.x(), position.y(), position.z(), velocity.x(), velocity.y(),
      velocity.z(), gyrobias.x(), gyrobias.y(), gyrobias.z(), accelbias.x(),
      accelbias.y(), accelbias.z();

  Eigen::Matrix<double, 15, 15> prior_covariance;
  prior_covariance.setIdentity();
  prior_covariance *= 1e-9;

  auto prior = AbsoluteImuState3DStampedConstraint::make_shared(
      "SOURCE", orientation, position, velocity, gyrobias, accelbias, mean,
      prior_covariance);
}

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
  beam_variables::GyroscopeBias3DStamped dummy_imu_bias_gyro;
  beam_variables::AccelerationBias3DStamped dummy_imu_bias_accel;
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
          dynamic_cast<const beam_variables::GyroscopeBias3DStamped&>(*iter);
      fuse_core::Variable::SharedPtr var_ptr =
          beam_variables::GyroscopeBias3DStamped::make_shared(var);
      graph.addVariable(var_ptr);
    } else if (iter->type() == dummy_imu_bias_accel.type()) {
      auto var =
          dynamic_cast<const beam_variables::AccelerationBias3DStamped&>(*iter);
      fuse_core::Variable::SharedPtr var_ptr =
          beam_variables::AccelerationBias3DStamped::make_shared(var);
      graph.addVariable(var_ptr);
    } else {
      return uuids;
    }
    uuids.push_back(iter->uuid());
  }
  return uuids;
}

void ExpectImuStateEq(const ImuState& IS1, const ImuState& IS2) {
  double tol = 1e-12;
  EXPECT_EQ(IS1.Stamp(), IS2.Stamp());
  EXPECT_NEAR(IS1.OrientationQuat().w(), IS2.OrientationQuat().w(), tol);
  EXPECT_NEAR(IS1.OrientationQuat().x(), IS2.OrientationQuat().x(), tol);
  EXPECT_NEAR(IS1.OrientationQuat().y(), IS2.OrientationQuat().y(), tol);
  EXPECT_NEAR(IS1.OrientationQuat().z(), IS2.OrientationQuat().z(), tol);
  EXPECT_NEAR(IS1.PositionVec()[0], IS2.PositionVec()[0], tol);
  EXPECT_NEAR(IS1.PositionVec()[1], IS2.PositionVec()[1], tol);
  EXPECT_NEAR(IS1.PositionVec()[2], IS2.PositionVec()[2], tol);
  EXPECT_NEAR(IS1.VelocityVec()[0], IS2.VelocityVec()[0], tol);
  EXPECT_NEAR(IS1.VelocityVec()[1], IS2.VelocityVec()[1], tol);
  EXPECT_NEAR(IS1.VelocityVec()[2], IS2.VelocityVec()[2], tol);
  EXPECT_NEAR(IS1.GyroBiasVec()[0], IS2.GyroBiasVec()[0], tol);
  EXPECT_NEAR(IS1.GyroBiasVec()[1], IS2.GyroBiasVec()[1], tol);
  EXPECT_NEAR(IS1.GyroBiasVec()[2], IS2.GyroBiasVec()[2], tol);
  EXPECT_NEAR(IS1.AccelBiasVec()[0], IS2.AccelBiasVec()[0], tol);
  EXPECT_NEAR(IS1.AccelBiasVec()[1], IS2.AccelBiasVec()[1], tol);
  EXPECT_NEAR(IS1.AccelBiasVec()[2], IS2.AccelBiasVec()[2], tol);
}

void ExpectImuStateNear(const ImuState& IS1, const ImuState& IS2) {
  EXPECT_EQ(IS1.Stamp(), IS2.Stamp());
  EXPECT_NEAR(IS1.OrientationQuat().w(), IS2.OrientationQuat().w(), 1e-6);
  EXPECT_NEAR(IS1.OrientationQuat().x(), IS2.OrientationQuat().x(), 1e-6);
  EXPECT_NEAR(IS1.OrientationQuat().y(), IS2.OrientationQuat().y(), 1e-6);
  EXPECT_NEAR(IS1.OrientationQuat().z(), IS2.OrientationQuat().z(), 1e-6);
  EXPECT_NEAR(IS1.PositionVec()[0], IS2.PositionVec()[0], 1e-3);
  EXPECT_NEAR(IS1.PositionVec()[1], IS2.PositionVec()[1], 1e-3);
  EXPECT_NEAR(IS1.PositionVec()[2], IS2.PositionVec()[2], 1e-4);
  EXPECT_NEAR(IS1.VelocityVec()[0], IS2.VelocityVec()[0], 1e-3);
  EXPECT_NEAR(IS1.VelocityVec()[1], IS2.VelocityVec()[1], 1e-3);
  EXPECT_NEAR(IS1.VelocityVec()[2], IS2.VelocityVec()[2], 1e-4);
  EXPECT_NEAR(IS1.GyroBiasVec()[0], IS2.GyroBiasVec()[0], 1e-9);
  EXPECT_NEAR(IS1.GyroBiasVec()[1], IS2.GyroBiasVec()[1], 1e-9);
  EXPECT_NEAR(IS1.GyroBiasVec()[2], IS2.GyroBiasVec()[2], 1e-9);
  EXPECT_NEAR(IS1.AccelBiasVec()[0], IS2.AccelBiasVec()[0], 1e-9);
  EXPECT_NEAR(IS1.AccelBiasVec()[1], IS2.AccelBiasVec()[1], 1e-9);
  EXPECT_NEAR(IS1.AccelBiasVec()[2], IS2.AccelBiasVec()[2], 1e-9);
}

void ExpectTransformsNear(const Eigen::Matrix4d& T1,
                          const Eigen::Matrix4d& T2) {
  Eigen::Quaterniond q1;
  Eigen::Vector3d p1;

  Eigen::Quaterniond q2;
  Eigen::Vector3d p2;

  beam::TransformMatrixToQuaternionAndTranslation(T1, q1, p1);
  beam::TransformMatrixToQuaternionAndTranslation(T2, q2, p2);

  EXPECT_NEAR(q1.w(), q2.w(), 1e-6);
  EXPECT_NEAR(q1.x(), q2.x(), 1e-6);
  EXPECT_NEAR(q1.y(), q2.y(), 1e-6);
  EXPECT_NEAR(q1.z(), q2.z(), 1e-6);
  EXPECT_NEAR(p1[0], p2[0], 1e-3);
  EXPECT_NEAR(p1[1], p2[1], 1e-3);
  EXPECT_NEAR(p1[2], p2[2], 1e-4);
}

TEST(ImuPreintegration, ImuState) {
  // create arbitrary state values
  Eigen::Quaterniond q_quat{Eigen::Quaterniond::UnitRandom()};
  Eigen::Vector3d p_vec{1, 2, 3};
  Eigen::Vector3d v_vec{0.1, 0.2, 0.3};
  Eigen::Vector3d bg_vec{0.001, 0.002, 0.003};
  Eigen::Vector3d ba_vec{0.0001, 0.0002, 0.0003};

  // instantiate class
  ImuState IS1(ros::Time(0), q_quat, p_vec, v_vec, bg_vec, ba_vec);

  // check fuse/beam variables getters
  EXPECT_EQ(IS1.Stamp(), ros::Time(0));
  EXPECT_EQ(IS1.Orientation().data()[0], q_quat.w());
  EXPECT_EQ(IS1.Orientation().data()[1], q_quat.x());
  EXPECT_EQ(IS1.Orientation().data()[2], q_quat.y());
  EXPECT_EQ(IS1.Orientation().data()[3], q_quat.z());
  EXPECT_EQ(IS1.Position().data()[0], p_vec[0]);
  EXPECT_EQ(IS1.Position().data()[1], p_vec[1]);
  EXPECT_EQ(IS1.Position().data()[2], p_vec[2]);
  EXPECT_EQ(IS1.Velocity().data()[0], v_vec[0]);
  EXPECT_EQ(IS1.Velocity().data()[1], v_vec[1]);
  EXPECT_EQ(IS1.Velocity().data()[2], v_vec[2]);
  EXPECT_EQ(IS1.GyroBias().data()[0], bg_vec[0]);
  EXPECT_EQ(IS1.GyroBias().data()[1], bg_vec[1]);
  EXPECT_EQ(IS1.GyroBias().data()[2], bg_vec[2]);
  EXPECT_EQ(IS1.AccelBias().data()[0], ba_vec[0]);
  EXPECT_EQ(IS1.AccelBias().data()[1], ba_vec[1]);
  EXPECT_EQ(IS1.AccelBias().data()[2], ba_vec[2]);

  // check quaternion/vector getters
  EXPECT_EQ(IS1.OrientationQuat().w(), q_quat.w());
  EXPECT_EQ(IS1.OrientationQuat().vec(), q_quat.vec());
  EXPECT_EQ(IS1.PositionVec(), p_vec);
  EXPECT_EQ(IS1.VelocityVec(), v_vec);
  EXPECT_EQ(IS1.GyroBiasVec(), bg_vec);
  EXPECT_EQ(IS1.AccelBiasVec(), ba_vec);

  // instantiate class with default state values
  ImuState IS2(ros::Time(1));

  // check default state values
  EXPECT_EQ(IS2.Stamp(), ros::Time(1));
  EXPECT_EQ(IS2.Orientation().data()[0], 1);
  EXPECT_EQ(IS2.Orientation().data()[1], 0);
  EXPECT_EQ(IS2.Orientation().data()[2], 0);
  EXPECT_EQ(IS2.Orientation().data()[3], 0);
  EXPECT_EQ(IS2.Position().data()[0], 0);
  EXPECT_EQ(IS2.Position().data()[1], 0);
  EXPECT_EQ(IS2.Position().data()[2], 0);
  EXPECT_EQ(IS2.Velocity().data()[0], 0);
  EXPECT_EQ(IS2.Velocity().data()[1], 0);
  EXPECT_EQ(IS2.Velocity().data()[2], 0);
  EXPECT_EQ(IS2.GyroBias().data()[0], 0);
  EXPECT_EQ(IS2.GyroBias().data()[1], 0);
  EXPECT_EQ(IS2.GyroBias().data()[2], 0);
  EXPECT_EQ(IS2.AccelBias().data()[0], 0);
  EXPECT_EQ(IS2.AccelBias().data()[1], 0);
  EXPECT_EQ(IS2.AccelBias().data()[2], 0);

  // check quaternion/vector setters
  IS2.SetOrientation(q_quat);
  IS2.SetPosition(p_vec);
  IS2.SetVelocity(v_vec);
  IS2.SetGyroBias(bg_vec);
  IS2.SetAccelBias(ba_vec);

  EXPECT_EQ(IS2.Orientation().data()[0], q_quat.w());
  EXPECT_EQ(IS2.Orientation().data()[1], q_quat.x());
  EXPECT_EQ(IS2.Orientation().data()[2], q_quat.y());
  EXPECT_EQ(IS2.Orientation().data()[3], q_quat.z());
  EXPECT_EQ(IS2.Position().data()[0], p_vec[0]);
  EXPECT_EQ(IS2.Position().data()[1], p_vec[1]);
  EXPECT_EQ(IS2.Position().data()[2], p_vec[2]);
  EXPECT_EQ(IS2.Velocity().data()[0], v_vec[0]);
  EXPECT_EQ(IS2.Velocity().data()[1], v_vec[1]);
  EXPECT_EQ(IS2.Velocity().data()[2], v_vec[2]);
  EXPECT_EQ(IS2.GyroBias().data()[0], bg_vec[0]);
  EXPECT_EQ(IS2.GyroBias().data()[1], bg_vec[1]);
  EXPECT_EQ(IS2.GyroBias().data()[2], bg_vec[2]);
  EXPECT_EQ(IS2.AccelBias().data()[0], ba_vec[0]);
  EXPECT_EQ(IS2.AccelBias().data()[1], ba_vec[1]);
  EXPECT_EQ(IS2.AccelBias().data()[2], ba_vec[2]);

  // check array setters
  IS2.SetOrientation(IS1.Orientation().data());
  IS2.SetPosition(IS1.Position().data());
  IS2.SetVelocity(IS1.Velocity().data());
  IS2.SetGyroBias(IS1.GyroBias().data());
  IS2.SetAccelBias(IS1.AccelBias().data());

  EXPECT_EQ(IS2.Orientation().data()[0], q_quat.w());
  EXPECT_EQ(IS2.Orientation().data()[1], q_quat.x());
  EXPECT_EQ(IS2.Orientation().data()[2], q_quat.y());
  EXPECT_EQ(IS2.Orientation().data()[3], q_quat.z());
  EXPECT_EQ(IS2.Position().data()[0], p_vec[0]);
  EXPECT_EQ(IS2.Position().data()[1], p_vec[1]);
  EXPECT_EQ(IS2.Position().data()[2], p_vec[2]);
  EXPECT_EQ(IS2.Velocity().data()[0], v_vec[0]);
  EXPECT_EQ(IS2.Velocity().data()[1], v_vec[1]);
  EXPECT_EQ(IS2.Velocity().data()[2], v_vec[2]);
  EXPECT_EQ(IS2.GyroBias().data()[0], bg_vec[0]);
  EXPECT_EQ(IS2.GyroBias().data()[1], bg_vec[1]);
  EXPECT_EQ(IS2.GyroBias().data()[2], bg_vec[2]);
  EXPECT_EQ(IS2.AccelBias().data()[0], ba_vec[0]);
  EXPECT_EQ(IS2.AccelBias().data()[1], ba_vec[1]);
  EXPECT_EQ(IS2.AccelBias().data()[2], ba_vec[2]);

  // check scalar setters
  IS2.SetOrientation(q_quat.w(), q_quat.x(), q_quat.y(), q_quat.z());
  IS2.SetPosition(p_vec[0], p_vec[1], p_vec[2]);
  IS2.SetVelocity(v_vec[0], v_vec[1], v_vec[2]);
  IS2.SetGyroBias(bg_vec[0], bg_vec[1], bg_vec[2]);
  IS2.SetAccelBias(ba_vec[0], ba_vec[1], ba_vec[2]);

  EXPECT_EQ(IS2.Orientation().data()[0], q_quat.w());
  EXPECT_EQ(IS2.Orientation().data()[1], q_quat.x());
  EXPECT_EQ(IS2.Orientation().data()[2], q_quat.y());
  EXPECT_EQ(IS2.Orientation().data()[3], q_quat.z());
  EXPECT_EQ(IS2.Position().data()[0], p_vec[0]);
  EXPECT_EQ(IS2.Position().data()[1], p_vec[1]);
  EXPECT_EQ(IS2.Position().data()[2], p_vec[2]);
  EXPECT_EQ(IS2.Velocity().data()[0], v_vec[0]);
  EXPECT_EQ(IS2.Velocity().data()[1], v_vec[1]);
  EXPECT_EQ(IS2.Velocity().data()[2], v_vec[2]);
  EXPECT_EQ(IS2.GyroBias().data()[0], bg_vec[0]);
  EXPECT_EQ(IS2.GyroBias().data()[1], bg_vec[1]);
  EXPECT_EQ(IS2.GyroBias().data()[2], bg_vec[2]);
  EXPECT_EQ(IS2.AccelBias().data()[0], ba_vec[0]);
  EXPECT_EQ(IS2.AccelBias().data()[1], ba_vec[1]);
  EXPECT_EQ(IS2.AccelBias().data()[2], ba_vec[2]);
}

TEST(ImuPreintegration, Simple2StateFG) {
  // create two imu states
  Data data;
  ImuState IS1 = data.IS1;
  ImuState IS2 = data.IS2;

  // assume small change in gyro and accel bias
  IS2.SetGyroBias(4e-5, 5e-5, 6e-5);
  IS2.SetAccelBias(1e-5, 2e-5, 3e-5);

  // Create the graph
  fuse_graphs::HashGraph graph;

  // Add variables
  fuse_variables::Orientation3DStamped::SharedPtr o1 =
      fuse_variables::Orientation3DStamped::make_shared(IS1.Orientation());
  fuse_variables::Position3DStamped::SharedPtr p1 =
      fuse_variables::Position3DStamped::make_shared(IS1.Position());
  fuse_variables::VelocityLinear3DStamped::SharedPtr v1 =
      fuse_variables::VelocityLinear3DStamped::make_shared(IS1.Velocity());
  beam_variables::GyroscopeBias3DStamped::SharedPtr bg1 =
      beam_variables::GyroscopeBias3DStamped::make_shared(IS1.GyroBias());
  beam_variables::AccelerationBias3DStamped::SharedPtr ba1 =
      beam_variables::AccelerationBias3DStamped::make_shared(IS1.AccelBias());

  fuse_variables::Orientation3DStamped::SharedPtr o2 =
      fuse_variables::Orientation3DStamped::make_shared(IS2.Orientation());
  fuse_variables::Position3DStamped::SharedPtr p2 =
      fuse_variables::Position3DStamped::make_shared(IS2.Position());
  fuse_variables::VelocityLinear3DStamped::SharedPtr v2 =
      fuse_variables::VelocityLinear3DStamped::make_shared(IS2.Velocity());
  beam_variables::GyroscopeBias3DStamped::SharedPtr bg2 =
      beam_variables::GyroscopeBias3DStamped::make_shared(IS2.GyroBias());
  beam_variables::AccelerationBias3DStamped::SharedPtr ba2 =
      beam_variables::AccelerationBias3DStamped::make_shared(IS2.AccelBias());

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

  // calculate relative state detla
  Eigen::Matrix<double, 16, 1> delta = CalculateRelativeStateDelta(IS1, IS2);

  // create covariance
  Eigen::Matrix<double, 15, 15> covariance;
  covariance.setIdentity();
  covariance *= 0.1;

  // Add relative constraint
  auto constraint = CreateRelativeConstraint(
      *o1, *p1, *v1, *bg1, *ba1, *o2, *p2, *v2, *bg2, *ba2, delta, covariance);
  graph.addConstraint(constraint);

  // Optimize the constraints and variables.
  graph.optimize();
  for (int i = 0; i < 4; i++) {
    EXPECT_EQ(o1->data()[i], IS1.Orientation().data()[i]);
    EXPECT_EQ(o2->data()[i], IS2.Orientation().data()[i]);
  }
  for (int i = 0; i < 3; i++) {
    EXPECT_EQ(p1->data()[i], IS1.Position().data()[i]);
    EXPECT_EQ(p2->data()[i], IS2.Position().data()[i]);
  }
  for (int i = 0; i < 3; i++) {
    EXPECT_EQ(v1->data()[i], IS1.Velocity().data()[i]);
    EXPECT_EQ(v2->data()[i], IS2.Velocity().data()[i]);
  }
  for (int i = 0; i < 3; i++) {
    EXPECT_EQ(bg1->data()[i], IS1.GyroBias().data()[i]);
    EXPECT_EQ(bg2->data()[i], IS2.GyroBias().data()[i]);
  }
  for (int i = 0; i < 3; i++) {
    EXPECT_EQ(ba1->data()[i], IS1.AccelBias().data()[i]);
    EXPECT_EQ(ba2->data()[i], IS2.AccelBias().data()[i]);
  }
}

class ImuPreintegration_ZeroNoiseZeroBias : public ::testing::Test {
 public:
  void SetUp() override {
    // set intrinsic noise of imu to zero
    params.cov_gyro_noise.setZero();
    params.cov_accel_noise.setZero();
    params.cov_gyro_bias.setZero();
    params.cov_accel_bias.setZero();

    // instantiate preintegration class with zero noise. By default,
    // bias terms (i.e. bg, ba) are set to zero
    imu_preintegration = std::make_unique<ImuPreintegration>(params);

    // populate ImuPreintegration with synthetic imu measurements
    for (beam_common::IMUData msg : data.imu_data_gt)
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
    params.prior_noise = 0;
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
  ImuState IS_default(t_start);
  ImuState IS_start_default = imu_preintegration->GetImuState();
  ExpectImuStateEq(IS_start_default, IS_default);

  // check optional initialization
  imu_preintegration->SetStart(t_start, o_start, p_start, v_start);
  ImuState IS_start = imu_preintegration->GetImuState();
  ExpectImuStateEq(IS_start, IS1);

  /**
   * PredictState() functionality
   */

  // populate Preintegrator class with imu preintegration deltas
  // from data class
  beam_common::PreIntegrator pre_integrator_12;
  pre_integrator_12.delta.t = data.delta_t_12;
  pre_integrator_12.delta.q = data.delta_q_12;
  pre_integrator_12.delta.p = data.delta_p_12;
  pre_integrator_12.delta.v = data.delta_v_12;

  beam_common::PreIntegrator pre_integrator_23;
  pre_integrator_23.delta.t = data.delta_t_23;
  pre_integrator_23.delta.q = data.delta_q_23;
  pre_integrator_23.delta.p = data.delta_p_23;
  pre_integrator_23.delta.v = data.delta_v_23;

  // predict middle and end imu state using relative change-in-motion ground
  // truth
  ImuState IS_middle_predict =
      imu_preintegration->PredictState(pre_integrator_12, IS_start);
  ImuState IS_end_predict =
      imu_preintegration->PredictState(pre_integrator_23, IS_middle_predict);

  // check
  ExpectImuStateEq(IS_middle_predict, IS2);
  ExpectImuStateEq(IS_end_predict, IS3);

  /**
   * CalculateRelativeChange() functionality
   */

  ExpectImuStateEq(imu_preintegration->GetImuState(), IS1);
  auto delta_start_end = imu_preintegration->CalculateRelativeChange(IS3);
  EXPECT_TRUE(
      delta_start_end.isApprox(CalculateRelativeStateDelta(IS1, IS3), 1e-6));

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
  ImuState IS_end = imu_preintegration->GetImuState();

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

  // get variables from graph
  auto o1 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph.getVariable(IS_start.Orientation().uuid()));
  auto o3 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph.getVariable(IS_end.Orientation().uuid()));
  auto p1 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph.getVariable(IS_start.Position().uuid()));
  auto p3 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph.getVariable(IS_end.Position().uuid()));
  auto v1 = dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(
      graph.getVariable(IS_start.Velocity().uuid()));
  auto v3 = dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(
      graph.getVariable(IS_end.Velocity().uuid()));
  auto bg1 = dynamic_cast<const beam_variables::GyroscopeBias3DStamped&>(
      graph.getVariable(IS_start.GyroBias().uuid()));
  auto bg3 = dynamic_cast<const beam_variables::GyroscopeBias3DStamped&>(
      graph.getVariable(IS_end.GyroBias().uuid()));
  auto ba1 = dynamic_cast<const beam_variables::AccelerationBias3DStamped&>(
      graph.getVariable(IS_start.AccelBias().uuid()));
  auto ba3 = dynamic_cast<const beam_variables::AccelerationBias3DStamped&>(
      graph.getVariable(IS_end.AccelBias().uuid()));

  // check
  for (int i = 0; i < 4; i++) {
    EXPECT_NEAR(o1.data()[i], IS1.Orientation().data()[i], 1e-6);
    EXPECT_NEAR(o3.data()[i], IS3.Orientation().data()[i], 1e-6);
  }

  for (int i = 0; i < 2; i++) {
    EXPECT_NEAR(p1.data()[i], IS1.Position().data()[i], 1e-3);
    EXPECT_NEAR(p3.data()[i], IS3.Position().data()[i], 1e-3);
  }
  EXPECT_NEAR(p1.data()[2], IS1.Position().data()[2], 1e-4);
  EXPECT_NEAR(p3.data()[2], IS3.Position().data()[2], 1e-4);

  for (int i = 0; i < 2; i++) {
    EXPECT_NEAR(v1.data()[i], IS1.Velocity().data()[i], 1e-3);
    EXPECT_NEAR(v3.data()[i], IS3.Velocity().data()[i], 1e-3);
  }
  EXPECT_NEAR(v1.data()[2], IS1.Velocity().data()[2], 1e-4);
  EXPECT_NEAR(v3.data()[2], IS3.Velocity().data()[2], 1e-4);

  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(bg1.data()[i], IS1.GyroBias().data()[i], 1e-9);
    EXPECT_NEAR(bg3.data()[i], IS3.GyroBias().data()[i], 1e-9);
  }
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(ba1.data()[i], IS1.AccelBias().data()[i], 1e-9);
    EXPECT_NEAR(ba3.data()[i], IS3.AccelBias().data()[i], 1e-9);
  }
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

  // for testing, call GetPose() from start to middle
  for (int i = t_start.toSec() + 1; i - 1 < t_middle.toSec(); i++) {
    Eigen::Matrix4d dummy_T_WORLD_IMU;
    imu_preintegration->GetPose(dummy_T_WORLD_IMU, ros::Time(i));
  }

  // generate transactions, taking start, middle, and end as key frames

  auto transaction1 =
      imu_preintegration->RegisterNewImuPreintegratedFactor(t_middle);
  auto transaction2 =
      imu_preintegration->RegisterNewImuPreintegratedFactor(t_end);

  // create graph
  fuse_graphs::HashGraph graph;

  // add transactions
  graph.update(*transaction1);
  graph.optimize();

  graph.update(*transaction2);
  graph.optimize();

  // get variables from graph
  auto o1 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph.getVariable(IS1.Orientation().uuid()));
  auto o2 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph.getVariable(IS2.Orientation().uuid()));
  auto o3 = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
      graph.getVariable(IS3.Orientation().uuid()));
  auto p1 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph.getVariable(IS1.Position().uuid()));
  auto p2 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph.getVariable(IS2.Position().uuid()));
  auto p3 = dynamic_cast<const fuse_variables::Position3DStamped&>(
      graph.getVariable(IS3.Position().uuid()));
  auto v1 = dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(
      graph.getVariable(IS1.Velocity().uuid()));
  auto v2 = dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(
      graph.getVariable(IS2.Velocity().uuid()));
  auto v3 = dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(
      graph.getVariable(IS3.Velocity().uuid()));
  auto bg1 = dynamic_cast<const beam_variables::GyroscopeBias3DStamped&>(
      graph.getVariable(IS1.GyroBias().uuid()));
  auto bg2 = dynamic_cast<const beam_variables::GyroscopeBias3DStamped&>(
      graph.getVariable(IS2.GyroBias().uuid()));
  auto bg3 = dynamic_cast<const beam_variables::GyroscopeBias3DStamped&>(
      graph.getVariable(IS3.GyroBias().uuid()));
  auto ba1 = dynamic_cast<const beam_variables::AccelerationBias3DStamped&>(
      graph.getVariable(IS1.AccelBias().uuid()));
  auto ba2 = dynamic_cast<const beam_variables::AccelerationBias3DStamped&>(
      graph.getVariable(IS2.AccelBias().uuid()));
  auto ba3 = dynamic_cast<const beam_variables::AccelerationBias3DStamped&>(
      graph.getVariable(IS3.AccelBias().uuid()));

  // check
  for (int i = 0; i < 4; i++) {
    EXPECT_NEAR(o1.data()[i], IS1.Orientation().data()[i], 1e-6);
    EXPECT_NEAR(o2.data()[i], IS2.Orientation().data()[i], 1e-6);
    EXPECT_NEAR(o3.data()[i], IS3.Orientation().data()[i], 1e-6);
  }

  for (int i = 0; i < 2; i++) {
    EXPECT_NEAR(p1.data()[i], IS1.Position().data()[i], 1e-3);
    EXPECT_NEAR(p2.data()[i], IS2.Position().data()[i], 1e-3);
    EXPECT_NEAR(p3.data()[i], IS3.Position().data()[i], 1e-3);
  }
  EXPECT_NEAR(p1.data()[2], IS1.Position().data()[2], 1e-4);
  EXPECT_NEAR(p2.data()[2], IS2.Position().data()[2], 1e-4);
  EXPECT_NEAR(p3.data()[2], IS3.Position().data()[2], 1e-4);

  for (int i = 0; i < 2; i++) {
    EXPECT_NEAR(v1.data()[i], IS1.Velocity().data()[i], 1e-3);
    EXPECT_NEAR(v2.data()[i], IS2.Velocity().data()[i], 1e-3);
    EXPECT_NEAR(v3.data()[i], IS3.Velocity().data()[i], 1e-3);
  }
  EXPECT_NEAR(v1.data()[2], IS1.Velocity().data()[2], 1e-4);
  EXPECT_NEAR(v2.data()[2], IS2.Velocity().data()[2], 1e-4);
  EXPECT_NEAR(v3.data()[2], IS3.Velocity().data()[2], 1e-4);

  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(bg1.data()[i], IS1.GyroBias().data()[i], 1e-9);
    EXPECT_NEAR(bg2.data()[i], IS2.GyroBias().data()[i], 1e-9);
    EXPECT_NEAR(bg3.data()[i], IS3.GyroBias().data()[i], 1e-9);
  }
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(ba1.data()[i], IS1.AccelBias().data()[i], 1e-9);
    EXPECT_NEAR(ba2.data()[i], IS2.AccelBias().data()[i], 1e-9);
    EXPECT_NEAR(ba3.data()[i], IS3.AccelBias().data()[i], 1e-9);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
