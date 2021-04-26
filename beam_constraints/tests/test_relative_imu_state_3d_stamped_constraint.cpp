#include <utility>
#include <vector>

#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/eigen_gtest.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <gtest/gtest.h>

#include <beam_constraints/frame_to_frame/relative_imu_state_3d_stamped_constraint.h>
#include <beam_constraints/global/absolute_constraint.h>

class Data {
public:
  Data() {
    device_id = fuse_core::uuid::generate("r5d4");

    orientation1 = fuse_variables::Orientation3DStamped::make_shared(
        ros::Time(1234, 5678), device_id);
    velocity1 = fuse_variables::VelocityLinear3DStamped::make_shared(
        ros::Time(1234, 5678), device_id);
    position1 = fuse_variables::Position3DStamped::make_shared(
        ros::Time(1234, 5678), device_id);
    accelbias1 = beam_variables::ImuBiasStamped::make_shared(
        ros::Time(1234, 5678), device_id);
    gyrobias1 = beam_variables::ImuBiasStamped::make_shared(
        ros::Time(1234, 5678), device_id);

    orientation2 = fuse_variables::Orientation3DStamped::make_shared(
        ros::Time(1235, 5678), device_id);
    velocity2 = fuse_variables::VelocityLinear3DStamped::make_shared(
        ros::Time(1235, 5678), device_id);
    position2 = fuse_variables::Position3DStamped::make_shared(
        ros::Time(1235, 5678), device_id);
    accelbias2 = beam_variables::ImuBiasStamped::make_shared(
        ros::Time(1234, 5678), device_id);
    gyrobias2 = beam_variables::ImuBiasStamped::make_shared(
        ros::Time(1234, 5678), device_id);

    delta << 0.988, 0.094, 0.079, 0.094, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 0.1, 0.2,
        0.3, 0.1, 0.2, 0.3;

    // clang-format off
    // Generated PD matrix using Octave: R = rand(15, 15); A = R * R' (use format long g to get the required precision)
    cov << 
      5.5165988279, 4.0920785731, 4.5353194492, 3.9147668573, 3.4503533721, 4.0809815566, 3.6977042284, 4.6219832603, 4.7697473290, 4.6700232116, 3.0295792180, 3.4228667454, 5.3123614126, 3.9579995536, 3.9758866470,
      4.0920785731, 4.9912686190, 3.2719101722, 3.6587999231, 2.8949346773, 4.1386316339, 3.3628533116, 3.2393099362, 4.4176311649, 4.1573976182, 2.5700883009, 3.1086510531, 4.3918109384, 3.2158738411, 4.0209533734,
      4.5353194492, 3.2719101722, 5.3352853791, 4.2456729101, 3.3104551942, 3.2019919287, 3.9431094640, 4.1156702411, 4.8226689134, 4.6960233501, 3.6300629881, 3.4161050839, 5.0461351255, 4.3989509370, 3.2300508874,
      3.9147668573, 3.6587999231, 4.2456729101, 4.7566000840, 3.2223410183, 3.3787324525, 3.2176565831, 3.5363966070, 4.8439417530, 3.9209165267, 2.7628716045, 3.1120315851, 4.6932292978, 3.5715506578, 3.5515560550,
      3.4503533721, 2.8949346773, 3.3104551942, 3.2223410183, 3.1548446899, 2.9983788638, 2.7492381552, 3.1539654643, 3.6707299955, 3.1325416500, 2.5090056117, 2.2634890365, 3.8945441300, 2.5589761003, 2.9551312449,
      4.0809815566, 4.1386316339, 3.2019919287, 3.3787324525, 2.9983788638, 4.8158857490, 2.8312898852, 3.4854024003, 4.2220605650, 4.5021390274, 2.8471831112, 2.9159393801, 3.9622769359, 3.4269865189, 4.1649514254,
      3.6977042284, 3.3628533116, 3.9431094640, 3.2176565831, 2.7492381552, 2.8312898852, 4.6461333896, 3.7925918936, 4.8257327333, 3.7641556270, 3.1444868558, 3.2152233661, 4.2080836233, 3.8009955144, 2.5738179376,
      4.6219832603, 3.2393099362, 4.1156702411, 3.5363966070, 3.1539654643, 3.4854024003, 3.7925918936, 5.0794711253, 4.9846867932, 4.0662620555, 3.4390257612, 3.1657614352, 4.4588984789, 3.8174463423, 3.4959320122,
      4.7697473290, 4.4176311649, 4.8226689134, 4.8439417530, 3.6707299955, 4.2220605650, 4.8257327333, 4.9846867932, 6.5621161761, 5.1442730425, 3.8398093098, 3.8948878877, 5.7955658147, 4.8876594732, 4.4084558238,
      4.6700232116, 4.1573976182, 4.6960233501, 3.9209165267, 3.1325416500, 4.5021390274, 3.7641556270, 4.0662620555, 5.1442730425, 6.2415142871, 3.5054322622, 4.0392086198, 4.9476688261, 4.5164750574, 4.4459634520,
      3.0295792180, 2.5700883009, 3.6300629881, 2.7628716045, 2.5090056117, 2.8471831112, 3.1444868558, 3.4390257612, 3.8398093098, 3.5054322622, 3.7180570703, 2.4676128781, 3.1521821278, 3.4139055574, 2.6214054991,
      3.4228667454, 3.1086510531, 3.4161050839, 3.1120315851, 2.2634890365, 2.9159393801, 3.2152233661, 3.1657614352, 3.8948878877, 4.0392086198, 2.4676128781, 3.5666671828, 3.3938678992, 2.8491591911, 3.0973321995,
      5.3123614126, 4.3918109384, 5.0461351255, 4.6932292978, 3.8945441300, 3.9622769359, 4.2080836233, 4.4588984789, 5.7955658147, 4.9476688261, 3.1521821278, 3.3938678992, 7.1778928469, 4.4108531520, 4.7505196128,
      3.9579995536, 3.2158738411, 4.3989509370, 3.5715506578, 2.5589761003, 3.4269865189, 3.8009955144, 3.8174463423, 4.8876594732, 4.5164750574, 3.4139055574, 2.8491591911, 4.4108531520, 5.2335226389, 3.3119965472,
      3.9758866470, 4.0209533734, 3.2300508874, 3.5515560550, 2.9551312449, 4.1649514254, 2.5738179376, 3.4959320122, 4.4084558238, 4.4459634520, 2.6214054991, 3.0973321995, 4.7505196128, 3.3119965472, 4.9944254214;
    // clang-format on

    relative_imu_state_constraint = std::make_shared<
        beam_constraints::frame_to_frame::RelativeImuState3DStampedConstraint>(
        "test", *orientation1, *velocity1, *position1, *accelbias1, *gyrobias1,
        *orientation2, *velocity2, *position2, *accelbias2, *gyrobias2, delta,
        cov);
  }
  fuse_variables::Orientation3DStamped::SharedPtr orientation1;
  fuse_variables::VelocityLinear3DStamped::SharedPtr velocity1;
  fuse_variables::Position3DStamped::SharedPtr position1;
  beam_variables::ImuBiasStamped::SharedPtr accelbias1;
  beam_variables::ImuBiasStamped::SharedPtr gyrobias1;
  fuse_variables::Orientation3DStamped::SharedPtr orientation2;
  fuse_variables::VelocityLinear3DStamped::SharedPtr velocity2;
  fuse_variables::Position3DStamped::SharedPtr position2;
  beam_variables::ImuBiasStamped::SharedPtr accelbias2;
  beam_variables::ImuBiasStamped::SharedPtr gyrobias2;

  Eigen::Matrix<double, 16, 1> delta;
  Eigen::Matrix<double, 15, 15> cov;

  beam_constraints::frame_to_frame::RelativeImuState3DStampedConstraint::
      SharedPtr relative_imu_state_constraint;

private:
  fuse_core::UUID device_id;
};

Data data_;

TEST(RelativeImuState3DStampedConstraint, Constructor) {
  EXPECT_NO_THROW(*data_.relative_imu_state_constraint);
}

TEST(RelativeImuState3DStampedConstraint, Covariance) {
  Eigen::Matrix<double, 15, 15> expected_cov = data_.cov;
  Eigen::Matrix<double, 15, 15> expected_sqrt_info =
      data_.cov.inverse().llt().matrixU();

  auto constraint = *data_.relative_imu_state_constraint;

  EXPECT_MATRIX_NEAR(expected_cov, constraint.covariance(), 1.0e-9);
  EXPECT_MATRIX_NEAR(expected_sqrt_info, constraint.sqrtInformation(), 1.0e-9);
}

TEST(RelativeImuState3DStampedConstraint, Optimization) {
  /*
  Optimize a two node system, where:
  1) An absolute pose constraint is generated at the origin of the world frame
  2) An absolute linear velocity constraint is generated at the origin of
  the world frame
  3) An absolute bias constraint is generated at the origin of the world frame
  3) A relative imu state constraint is generated
  */

  // Create imu state 1
  auto orientation1 = fuse_variables::Orientation3DStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  orientation1->w() = 0.952;
  orientation1->x() = 0.038;
  orientation1->y() = -0.189;
  orientation1->z() = 0.239;

  auto velocity1 = fuse_variables::VelocityLinear3DStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  velocity1->x() = 1.5;
  velocity1->y() = -3.0;
  velocity1->z() = 10.0;

  auto position1 = fuse_variables::Position3DStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  position1->x() = 1.5;
  position1->y() = -3.0;
  position1->z() = 10.0;

  auto accelbias1 = beam_variables::ImuBiasStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  accelbias1->x() = 0.15;
  accelbias1->y() = -0.30;
  accelbias1->z() = 1.0;

  auto gyrobias1 = beam_variables::ImuBiasStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  gyrobias1->x() = 0.15;
  gyrobias1->y() = -0.30;
  gyrobias1->z() = 1.0;

  // Create imu state 2
  auto orientation2 = fuse_variables::Orientation3DStamped::make_shared(
      ros::Time(2, 0), fuse_core::uuid::generate("spra"));
  orientation2->w() = 0.944;
  orientation2->x() = -0.128;
  orientation2->y() = 0.145;
  orientation2->z() = -0.269;

  auto velocity2 = fuse_variables::VelocityLinear3DStamped::make_shared(
      ros::Time(2, 0), fuse_core::uuid::generate("spra"));
  velocity2->x() = -1.5;
  velocity2->y() = 3.0;
  velocity2->z() = -10.0;

  auto position2 = fuse_variables::Position3DStamped::make_shared(
      ros::Time(2, 0), fuse_core::uuid::generate("spra"));
  position2->x() = -1.5;
  position2->y() = 3.0;
  position2->z() = -10.0;

  auto accelbias2 = beam_variables::ImuBiasStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  accelbias2->x() = 0.15;
  accelbias2->y() = -0.30;
  accelbias2->z() = 1.0;

  auto gyrobias2 = beam_variables::ImuBiasStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  gyrobias2->x() = 0.15;
  gyrobias2->y() = -0.30;
  gyrobias2->z() = 1.0;

  // Create an absolute pose constraint at the origin
  fuse_core::Vector7d mean_pose_origin;
  mean_pose_origin << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  fuse_core::Matrix6d cov_pose_origin = fuse_core::Matrix6d::Identity();
  auto prior_pose =
      fuse_constraints::AbsolutePose3DStampedConstraint::make_shared(
          "test", *position1, *orientation1, mean_pose_origin, cov_pose_origin);

  // Create an absolute linear velocity constraint at the origin
  fuse_core::Vector3d mean_vel_origin;
  mean_vel_origin << 0.0, 0.0, 0.0;
  fuse_core::Matrix3d cov_vel_origin = fuse_core::Matrix3d::Identity();
  auto prior_velocity =
      beam_constraints::global::AbsoluteVelocityLinear3DStampedConstraint::
          make_shared("test", *velocity1, mean_vel_origin, cov_vel_origin);

  // Create an absolute imu bias constraint at the origin
  fuse_core::Vector3d mean_accel_bias_origin;
  mean_accel_bias_origin << 0.0, 0.0, 0.0;
  fuse_core::Matrix3d cov_accel_bias_origin = fuse_core::Matrix3d::Identity();
  auto prior_accel_bias =
      beam_constraints::global::AbsoluteImuBias3DStampedConstraint::make_shared(
          "test", *accelbias1, mean_accel_bias_origin, cov_accel_bias_origin);

  fuse_core::Vector3d mean_gyro_bias_origin;
  mean_gyro_bias_origin << 0.0, 0.0, 0.0;
  fuse_core::Matrix3d cov_gyro_bias_origin = fuse_core::Matrix3d::Identity();
  auto prior_gyro_bias =
      beam_constraints::global::AbsoluteImuBias3DStampedConstraint::make_shared(
          "test", *gyrobias1, mean_gyro_bias_origin, cov_gyro_bias_origin);

  // Create a relative imu state constraint, assuming:
  // 1) a +1m change in position in the x direction
  // 2) a +1m/s change in velocity in the x direction
  // 2) a +0.1m/^2 change in accleration bias in the x direction
  // 2) a +0.1w change in gyro bias in the x direction
  Eigen::Matrix<double, 16, 1> mean_delta;
  mean_delta << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0.0, 0.0,
      0.1, 0.0, 0.0;
  Eigen::Matrix<double, 15, 15> cov_delta;
  cov_delta.setIdentity();
  auto relative_imu_state = beam_constraints::frame_to_frame::
      RelativeImuState3DStampedConstraint::make_shared(
          "test", *orientation1, *velocity1, *position1, *accelbias1,
          *gyrobias1, *orientation2, *velocity2, *position2, *accelbias2,
          *gyrobias2, mean_delta, cov_delta);

  // Build the problem
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
  ceres::Problem problem(problem_options);

  problem.AddParameterBlock(orientation1->data(), orientation1->size(),
                            orientation1->localParameterization());
  problem.AddParameterBlock(velocity1->data(), velocity1->size(),
                            velocity1->localParameterization());
  problem.AddParameterBlock(position1->data(), position1->size(),
                            position1->localParameterization());
  problem.AddParameterBlock(accelbias1->data(), accelbias1->size(),
                            accelbias1->localParameterization());
  problem.AddParameterBlock(gyrobias1->data(), gyrobias1->size(),
                            gyrobias1->localParameterization());

  problem.AddParameterBlock(orientation2->data(), orientation2->size(),
                            orientation2->localParameterization());
  problem.AddParameterBlock(velocity2->data(), velocity2->size(),
                            velocity2->localParameterization());
  problem.AddParameterBlock(position2->data(), position2->size(),
                            position2->localParameterization());
  problem.AddParameterBlock(accelbias2->data(), accelbias2->size(),
                            accelbias2->localParameterization());
  problem.AddParameterBlock(gyrobias2->data(), gyrobias2->size(),
                            gyrobias2->localParameterization());

  std::vector<double *> prior_pose_parameter_blocks;
  prior_pose_parameter_blocks.push_back(position1->data());
  prior_pose_parameter_blocks.push_back(orientation1->data());
  problem.AddResidualBlock(prior_pose->costFunction(),
                           prior_pose->lossFunction(),
                           prior_pose_parameter_blocks);

  std::vector<double *> prior_velocity_parameter_blocks;
  prior_velocity_parameter_blocks.push_back(velocity1->data());
  problem.AddResidualBlock(prior_velocity->costFunction(),
                           prior_velocity->lossFunction(),
                           prior_velocity_parameter_blocks);

  std::vector<double *> prior_accel_bias_parameter_blocks;
  prior_accel_bias_parameter_blocks.push_back(accelbias1->data());
  problem.AddResidualBlock(prior_accel_bias->costFunction(),
                           prior_accel_bias->lossFunction(),
                           prior_accel_bias_parameter_blocks);

  std::vector<double *> prior_gyro_bias_parameter_blocks;
  prior_gyro_bias_parameter_blocks.push_back(gyrobias1->data());
  problem.AddResidualBlock(prior_gyro_bias->costFunction(),
                           prior_gyro_bias->lossFunction(),
                           prior_gyro_bias_parameter_blocks);

  std::vector<double *> relative_imu_state_parameter_blocks;
  relative_imu_state_parameter_blocks.push_back(orientation1->data());
  relative_imu_state_parameter_blocks.push_back(velocity1->data());
  relative_imu_state_parameter_blocks.push_back(position1->data());
  relative_imu_state_parameter_blocks.push_back(accelbias1->data());
  relative_imu_state_parameter_blocks.push_back(gyrobias1->data());
  relative_imu_state_parameter_blocks.push_back(orientation2->data());
  relative_imu_state_parameter_blocks.push_back(velocity2->data());
  relative_imu_state_parameter_blocks.push_back(position2->data());
  relative_imu_state_parameter_blocks.push_back(accelbias2->data());
  relative_imu_state_parameter_blocks.push_back(gyrobias2->data());
  problem.AddResidualBlock(relative_imu_state->costFunction(),
                           relative_imu_state->lossFunction(),
                           relative_imu_state_parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(1.0, orientation1->w(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation1->x(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation1->y(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation1->z(), 1.0e-3);
  EXPECT_NEAR(0.0, velocity1->x(), 1.0e-5);
  EXPECT_NEAR(0.0, velocity1->y(), 1.0e-5);
  EXPECT_NEAR(0.0, velocity1->z(), 1.0e-5);
  EXPECT_NEAR(0.0, position1->x(), 1.0e-5);
  EXPECT_NEAR(0.0, position1->y(), 1.0e-5);
  EXPECT_NEAR(0.0, position1->z(), 1.0e-5);
  EXPECT_NEAR(0.0, accelbias1->x(), 1.0e-5);
  EXPECT_NEAR(0.0, accelbias1->y(), 1.0e-5);
  EXPECT_NEAR(0.0, accelbias1->z(), 1.0e-5);
  EXPECT_NEAR(0.0, gyrobias1->x(), 1.0e-5);
  EXPECT_NEAR(0.0, gyrobias1->y(), 1.0e-5);
  EXPECT_NEAR(0.0, gyrobias1->z(), 1.0e-5);
  EXPECT_NEAR(1.0, orientation2->w(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation2->x(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation2->y(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation2->z(), 1.0e-3);
  EXPECT_NEAR(1.0, velocity2->x(), 1.0e-5);
  EXPECT_NEAR(0.0, velocity2->y(), 1.0e-5);
  EXPECT_NEAR(0.0, velocity2->z(), 1.0e-5);
  EXPECT_NEAR(1.0, position2->x(), 1.0e-5);
  EXPECT_NEAR(0.0, position2->y(), 1.0e-5);
  EXPECT_NEAR(0.0, position2->z(), 1.0e-5);
  EXPECT_NEAR(0.1, accelbias2->x(), 1.0e-5);
  EXPECT_NEAR(0.0, accelbias2->y(), 1.0e-5);
  EXPECT_NEAR(0.0, accelbias2->z(), 1.0e-5);
  EXPECT_NEAR(0.1, gyrobias2->x(), 1.0e-5);
  EXPECT_NEAR(0.0, gyrobias2->y(), 1.0e-5);
  EXPECT_NEAR(0.0, gyrobias2->z(), 1.0e-5);

  // Compute the marginal covariance for imu state 1
  {
    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    covariance_blocks.emplace_back(orientation1->data(), orientation1->data());
    covariance_blocks.emplace_back(orientation1->data(), velocity1->data());
    covariance_blocks.emplace_back(orientation1->data(), position1->data());
    covariance_blocks.emplace_back(orientation1->data(), accelbias1->data());
    covariance_blocks.emplace_back(orientation1->data(), gyrobias1->data());
    covariance_blocks.emplace_back(velocity1->data(), velocity1->data());
    covariance_blocks.emplace_back(velocity1->data(), position1->data());
    covariance_blocks.emplace_back(velocity1->data(), accelbias1->data());
    covariance_blocks.emplace_back(velocity1->data(), gyrobias1->data());
    covariance_blocks.emplace_back(position1->data(), position1->data());
    covariance_blocks.emplace_back(position1->data(), accelbias1->data());
    covariance_blocks.emplace_back(position1->data(), gyrobias1->data());
    covariance_blocks.emplace_back(accelbias1->data(), accelbias1->data());
    covariance_blocks.emplace_back(accelbias1->data(), gyrobias1->data());
    covariance_blocks.emplace_back(gyrobias1->data(), gyrobias1->data());

    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    covariance.Compute(covariance_blocks, &problem);

    fuse_core::MatrixXd cov_or_or(orientation1->localSize(),
                                  orientation1->localSize());
    covariance.GetCovarianceBlockInTangentSpace(
        orientation1->data(), orientation1->data(), cov_or_or.data());

    fuse_core::MatrixXd cov_or_vel(orientation1->localSize(),
                                   velocity1->localSize());
    covariance.GetCovarianceBlockInTangentSpace(
        orientation1->data(), velocity1->data(), cov_or_vel.data());

    fuse_core::MatrixXd cov_or_pos(orientation1->localSize(),
                                   position1->localSize());
    covariance.GetCovarianceBlockInTangentSpace(
        orientation1->data(), position1->data(), cov_or_pos.data());

    fuse_core::MatrixXd cov_or_ba(orientation1->localSize(),
                                  accelbias1->localSize());
    covariance.GetCovarianceBlockInTangentSpace(
        orientation1->data(), accelbias1->data(), cov_or_ba.data());

    fuse_core::MatrixXd cov_or_bg(orientation1->localSize(),
                                  gyrobias1->localSize());
    covariance.GetCovarianceBlockInTangentSpace(
        orientation1->data(), gyrobias1->data(), cov_or_bg.data());

    fuse_core::MatrixXd cov_vel_vel(velocity1->size(), velocity1->size());
    covariance.GetCovarianceBlock(velocity1->data(), velocity1->data(),
                                  cov_vel_vel.data());

    fuse_core::MatrixXd cov_vel_pos(velocity1->size(), position1->size());
    covariance.GetCovarianceBlock(velocity1->data(), position1->data(),
                                  cov_vel_pos.data());

    fuse_core::MatrixXd cov_vel_ba(velocity1->size(), accelbias1->size());
    covariance.GetCovarianceBlock(velocity1->data(), accelbias1->data(),
                                  cov_vel_ba.data());

    fuse_core::MatrixXd cov_vel_bg(velocity1->size(), gyrobias1->size());
    covariance.GetCovarianceBlock(velocity1->data(), gyrobias1->data(),
                                  cov_vel_bg.data());

    fuse_core::MatrixXd cov_pos_pos(position1->size(), position1->size());
    covariance.GetCovarianceBlock(position1->data(), position1->data(),
                                  cov_pos_pos.data());

    fuse_core::MatrixXd cov_pos_ba(position1->size(), accelbias1->size());
    covariance.GetCovarianceBlock(position1->data(), accelbias1->data(),
                                  cov_pos_ba.data());

    fuse_core::MatrixXd cov_pos_bg(position1->size(), gyrobias1->size());
    covariance.GetCovarianceBlock(position1->data(), gyrobias1->data(),
                                  cov_pos_bg.data());

    fuse_core::MatrixXd cov_ba_ba(accelbias1->size(), accelbias1->size());
    covariance.GetCovarianceBlock(accelbias1->data(), accelbias1->data(),
                                  cov_ba_ba.data());

    fuse_core::MatrixXd cov_ba_bg(accelbias1->size(), gyrobias1->size());
    covariance.GetCovarianceBlock(accelbias1->data(), gyrobias1->data(),
                                  cov_ba_bg.data());

    fuse_core::MatrixXd cov_bg_bg(gyrobias1->size(), gyrobias1->size());
    covariance.GetCovarianceBlock(gyrobias1->data(), gyrobias1->data(),
                                  cov_bg_bg.data());

    // Assemble the full covariance from the covariance blocks
    Eigen::Matrix<double, 15, 15> actual_covariance;
    actual_covariance << cov_or_or, cov_or_vel, cov_or_pos, cov_or_ba,
        cov_or_bg, cov_or_vel.transpose(), cov_vel_vel, cov_vel_pos, cov_vel_ba,
        cov_vel_bg, cov_or_pos.transpose(), cov_vel_pos.transpose(),
        cov_pos_pos, cov_pos_ba, cov_pos_bg, cov_or_ba.transpose(),
        cov_vel_ba.transpose(), cov_pos_ba.transpose(), cov_ba_ba, cov_ba_bg,
        cov_or_bg.transpose(), cov_vel_bg.transpose(), cov_pos_bg.transpose(),
        cov_ba_bg.transpose(), cov_bg_bg;

    // Define the expected covariance
    Eigen::Matrix<double, 15, 15> expected_covariance;
    expected_covariance.setIdentity();

    EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-5);
  }

  // Compute the marginal covariance for imu state 2
  {
    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    covariance_blocks.emplace_back(orientation2->data(), orientation2->data());
    covariance_blocks.emplace_back(orientation2->data(), velocity2->data());
    covariance_blocks.emplace_back(orientation2->data(), position2->data());
    covariance_blocks.emplace_back(orientation2->data(), accelbias2->data());
    covariance_blocks.emplace_back(orientation2->data(), gyrobias2->data());
    covariance_blocks.emplace_back(velocity2->data(), velocity2->data());
    covariance_blocks.emplace_back(velocity2->data(), position2->data());
    covariance_blocks.emplace_back(velocity2->data(), accelbias2->data());
    covariance_blocks.emplace_back(velocity2->data(), gyrobias2->data());
    covariance_blocks.emplace_back(position2->data(), position2->data());
    covariance_blocks.emplace_back(position2->data(), accelbias2->data());
    covariance_blocks.emplace_back(position2->data(), gyrobias2->data());
    covariance_blocks.emplace_back(accelbias2->data(), accelbias2->data());
    covariance_blocks.emplace_back(accelbias2->data(), gyrobias2->data());
    covariance_blocks.emplace_back(gyrobias2->data(), gyrobias2->data());

    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    covariance.Compute(covariance_blocks, &problem);

    fuse_core::MatrixXd cov_or_or(orientation2->localSize(),
                                  orientation2->localSize());
    covariance.GetCovarianceBlockInTangentSpace(
        orientation2->data(), orientation2->data(), cov_or_or.data());

    fuse_core::MatrixXd cov_or_vel(orientation2->localSize(),
                                   velocity2->localSize());
    covariance.GetCovarianceBlockInTangentSpace(
        orientation2->data(), velocity2->data(), cov_or_vel.data());

    fuse_core::MatrixXd cov_or_pos(orientation2->localSize(),
                                   position2->localSize());
    covariance.GetCovarianceBlockInTangentSpace(
        orientation2->data(), position2->data(), cov_or_pos.data());

    fuse_core::MatrixXd cov_or_ba(orientation2->localSize(),
                                  accelbias2->localSize());
    covariance.GetCovarianceBlockInTangentSpace(
        orientation2->data(), accelbias2->data(), cov_or_ba.data());

    fuse_core::MatrixXd cov_or_bg(orientation2->localSize(),
                                  gyrobias2->localSize());
    covariance.GetCovarianceBlockInTangentSpace(
        orientation2->data(), gyrobias2->data(), cov_or_bg.data());

    fuse_core::MatrixXd cov_vel_vel(velocity2->size(), velocity2->size());
    covariance.GetCovarianceBlock(velocity2->data(), velocity2->data(),
                                  cov_vel_vel.data());

    fuse_core::MatrixXd cov_vel_pos(velocity2->size(), position2->size());
    covariance.GetCovarianceBlock(velocity2->data(), position2->data(),
                                  cov_vel_pos.data());

    fuse_core::MatrixXd cov_vel_ba(velocity2->size(), accelbias2->size());
    covariance.GetCovarianceBlock(velocity2->data(), accelbias2->data(),
                                  cov_vel_ba.data());

    fuse_core::MatrixXd cov_vel_bg(velocity2->size(), gyrobias2->size());
    covariance.GetCovarianceBlock(velocity2->data(), gyrobias2->data(),
                                  cov_vel_bg.data());

    fuse_core::MatrixXd cov_pos_pos(position2->size(), position2->size());
    covariance.GetCovarianceBlock(position2->data(), position2->data(),
                                  cov_pos_pos.data());

    fuse_core::MatrixXd cov_pos_ba(position2->size(), accelbias2->size());
    covariance.GetCovarianceBlock(position2->data(), accelbias2->data(),
                                  cov_pos_ba.data());

    fuse_core::MatrixXd cov_pos_bg(position2->size(), gyrobias2->size());
    covariance.GetCovarianceBlock(position2->data(), gyrobias2->data(),
                                  cov_pos_bg.data());

    fuse_core::MatrixXd cov_ba_ba(accelbias2->size(), accelbias2->size());
    covariance.GetCovarianceBlock(accelbias2->data(), accelbias2->data(),
                                  cov_ba_ba.data());

    fuse_core::MatrixXd cov_ba_bg(accelbias2->size(), gyrobias2->size());
    covariance.GetCovarianceBlock(accelbias2->data(), gyrobias2->data(),
                                  cov_ba_bg.data());

    fuse_core::MatrixXd cov_bg_bg(gyrobias2->size(), gyrobias2->size());
    covariance.GetCovarianceBlock(gyrobias2->data(), gyrobias2->data(),
                                  cov_bg_bg.data());

    // Assemble the full covariance from the covariance blocks
    Eigen::Matrix<double, 15, 15> actual_covariance;
    actual_covariance << cov_or_or, cov_or_vel, cov_or_pos, cov_or_ba,
        cov_or_bg, cov_or_vel.transpose(), cov_vel_vel, cov_vel_pos, cov_vel_ba,
        cov_vel_bg, cov_or_pos.transpose(), cov_vel_pos.transpose(),
        cov_pos_pos, cov_pos_ba, cov_pos_bg, cov_or_ba.transpose(),
        cov_vel_ba.transpose(), cov_pos_ba.transpose(), cov_ba_ba, cov_ba_bg,
        cov_or_bg.transpose(), cov_vel_bg.transpose(), cov_pos_bg.transpose(),
        cov_ba_bg.transpose(), cov_bg_bg;

    // clang-format off
    // Define the expected covariance
    Eigen::Matrix<double, 15, 15> expected_covariance;
    expected_covariance <<
        2,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  2,  0,  0,  0,  -1, 0,  0, -1,  0,  0,  0,  0,  0,  0,
        0,  0,  2,  0,  1,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  1,  0,  3,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,
        0,  -1, 0,  0,  0,  3,  0,  0,  1,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  2,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  1,  0,  1,  0,  0,  3,  0,  0,  0,  0,  0,  0,  0,
        0,  -1, 0,  0,  0,  1,  0,  0,  3,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  2,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  2,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  2,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  2,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  2,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  2;
    // clang-format on

    EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-5);
  }
}

TEST(RelativeImuState3DStampedConstraint, Serialization) {
  // Construct a constraint
  beam_constraints::frame_to_frame::RelativeImuState3DStampedConstraint
      expected("test", *(data_.orientation1), *(data_.velocity1),
               *(data_.position1), *(data_.accelbias1), *(data_.gyrobias1),
               *(data_.orientation2), *(data_.velocity2), *(data_.position2),
               *(data_.accelbias2), *(data_.gyrobias2), data_.delta, data_.cov);

  // Serialize the constraint into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new constraint from that same stream
  beam_constraints::frame_to_frame::RelativeImuState3DStampedConstraint actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.uuid(), actual.uuid());
  EXPECT_EQ(expected.variables(), actual.variables());
  EXPECT_MATRIX_EQ(expected.delta(), actual.delta());
  EXPECT_MATRIX_EQ(expected.sqrtInformation(), actual.sqrtInformation());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
