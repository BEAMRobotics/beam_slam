#pragma once

#include <utility>
#include <vector>

#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <fuse_core/eigen.h>
#include <fuse_core/eigen_gtest.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <gtest/gtest.h>

#include <bs_common/imu_state.h>
#include <bs_constraints/global/absolute_imu_state_3d_stamped_constraint.h>

class Data {
 public:
  Data() {
    // generate measurements on IMU state
    Eigen::Quaterniond q_quat(0.952, 0.038, -0.189, 0.239);
    Eigen::Vector3d p_vec(1.5, -3.0, 10.0);
    Eigen::Vector3d v_vec(1.5, -3.0, 10.0);
    Eigen::Vector3d bg_vec(0.15, -0.30, 1.0);
    Eigen::Vector3d ba_vec(0.15, -0.30, 1.0);

    // populate IMU state with measurements
    bs_common::ImuState tmp(ros::Time(1, 0), q_quat, p_vec, v_vec, bg_vec,
                            ba_vec);
    imu_state = std::move(tmp);

    // populate mean with arbitrary measurements
    mean << 1.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3,
        0.1, 0.2, 0.3;

    // clang-format off
    cov << 1.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9,  1.0,  1.1,  1.2,  1.3,  1.4,
           0.1, 2.0, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9, 0.8,  0.7,  0.6,  0.5,  0.4,  0.3,
           0.2, 1.5, 3.0, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9,  0.8,  0.7,  0.6,  0.5,  0.4,
           0.3, 1.4, 1.5, 4.0, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0,  0.9,  0.8,  0.7,  0.6,  0.5,
           0.4, 1.3, 1.4, 1.5, 5.0, 1.5, 1.4, 1.3, 1.2, 1.1,  1.0,  0.9,  0.8,  0.7,  0.6,
           0.5, 1.2, 1.3, 1.4, 1.5, 6.0, 1.5, 1.4, 1.3, 1.2,  1.1,  1.0,  0.9,  0.8,  0.7,
           0.6, 1.1, 1.2, 1.3, 1.4, 1.5, 7.0, 1.5, 1.4, 1.3,  1.2,  1.1,  1.0,  0.9,  0.8,
           0.7, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 8.0, 1.5, 1.4,  1.3,  1.2,  1.1,  1.0,  0.9,
           0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 9.0, 1.5,  1.4,  1.3,  1.2,  1.1,  1.0,
           0.9, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 10.0, 1.5,  1.4,  1.3,  1.2,  1.1,
           1.0, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5,  11.0, 1.5,  1.4,  1.3,  1.2,
           1.1, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4,  1.5,  12.0, 1.5,  1.4,  1.3,
           1.2, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3,  1.4,  1.5,  13.0, 1.5,  1.4,
           1.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2,  1.3,  1.4,  1.5,  14.0, 1.5,
           1.4, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1,  1.2,  1.3,  1.4,  1.5,  15.0;
    // clang-format on

    absolute_imu_state_constraint = std::make_shared<
        bs_constraints::global::AbsoluteImuState3DStampedConstraint>(
        "test", imu_state, mean, cov);
  }

  bs_common::ImuState imu_state;
  Eigen::Matrix<double, 16, 1> mean;
  Eigen::Matrix<double, 15, 15> cov;

  bs_constraints::global::AbsoluteImuState3DStampedConstraint::SharedPtr
      absolute_imu_state_constraint;
};

Data data_;

TEST(AbsoluteImuState3DStampedConstraint, Constructor) {
  EXPECT_NO_THROW(*data_.absolute_imu_state_constraint);
}

TEST(AbsoluteImuState3DStampedConstraint, Covariance) {
  Eigen::Matrix<double, 15, 15> expected_cov = data_.cov;
  Eigen::Matrix<double, 15, 15> expected_sqrt_info =
      data_.cov.inverse().llt().matrixU();

  auto constraint = *data_.absolute_imu_state_constraint;

  EXPECT_MATRIX_NEAR(expected_cov, constraint.covariance(), 1.0e-9);
  EXPECT_MATRIX_NEAR(expected_sqrt_info, constraint.sqrtInformation(), 1.0e-9);
}

TEST(AbsoluteImuState3DStampedConstraint, Optimization) {
  /*
  Optimize a single IMU state and single constraint, verify the
  expected value and covariance are generated. Create the variables
  */

  // extract fuse/beam variables
  auto orientation_variable = fuse_variables::Orientation3DStamped::make_shared(
      data_.imu_state.Orientation());

  auto position_variable = fuse_variables::Position3DStamped::make_shared(
      data_.imu_state.Position());

  auto velocity_variable = fuse_variables::VelocityLinear3DStamped::make_shared(
      data_.imu_state.Velocity());

  auto gyro_bias_variable = bs_variables::GyroscopeBias3DStamped::make_shared(
      data_.imu_state.GyroBias());

  auto accel_bias_variable =
      bs_variables::AccelerationBias3DStamped::make_shared(
          data_.imu_state.AccelBias());

  // Create an absolute pose constraint
  auto constraint =
      bs_constraints::global::AbsoluteImuState3DStampedConstraint::make_shared(
          "test", data_.imu_state, data_.mean, data_.cov);

  // Build the problem
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
  ceres::Problem problem(problem_options);

  problem.AddParameterBlock(orientation_variable->data(),
                            orientation_variable->size(),
                            orientation_variable->localParameterization());
  problem.AddParameterBlock(position_variable->data(),
                            position_variable->size(),
                            position_variable->localParameterization());
  problem.AddParameterBlock(velocity_variable->data(),
                            velocity_variable->size(),
                            velocity_variable->localParameterization());
  problem.AddParameterBlock(gyro_bias_variable->data(),
                            gyro_bias_variable->size(),
                            gyro_bias_variable->localParameterization());
  problem.AddParameterBlock(accel_bias_variable->data(),
                            accel_bias_variable->size(),
                            accel_bias_variable->localParameterization());

  std::vector<double *> parameter_blocks;
  parameter_blocks.push_back(orientation_variable->data());
  parameter_blocks.push_back(position_variable->data());
  parameter_blocks.push_back(velocity_variable->data());
  parameter_blocks.push_back(gyro_bias_variable->data());
  parameter_blocks.push_back(accel_bias_variable->data());

  problem.AddResidualBlock(constraint->costFunction(),
                           constraint->lossFunction(), parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(data_.mean[0], orientation_variable->w(), 1.0e-3);
  EXPECT_NEAR(data_.mean[1], orientation_variable->x(), 1.0e-3);
  EXPECT_NEAR(data_.mean[2], orientation_variable->y(), 1.0e-3);
  EXPECT_NEAR(data_.mean[3], orientation_variable->z(), 1.0e-3);
  EXPECT_NEAR(data_.mean[4], position_variable->x(), 1.0e-5);
  EXPECT_NEAR(data_.mean[5], position_variable->y(), 1.0e-5);
  EXPECT_NEAR(data_.mean[6], position_variable->z(), 1.0e-5);
  EXPECT_NEAR(data_.mean[7], velocity_variable->x(), 1.0e-5);
  EXPECT_NEAR(data_.mean[8], velocity_variable->y(), 1.0e-5);
  EXPECT_NEAR(data_.mean[9], velocity_variable->z(), 1.0e-5);
  EXPECT_NEAR(data_.mean[10], gyro_bias_variable->x(), 1.0e-5);
  EXPECT_NEAR(data_.mean[11], gyro_bias_variable->y(), 1.0e-5);
  EXPECT_NEAR(data_.mean[12], gyro_bias_variable->z(), 1.0e-5);
  EXPECT_NEAR(data_.mean[13], accel_bias_variable->x(), 1.0e-5);
  EXPECT_NEAR(data_.mean[14], accel_bias_variable->y(), 1.0e-5);
  EXPECT_NEAR(data_.mean[15], accel_bias_variable->z(), 1.0e-5);

  // Compute the covariance
  std::vector<std::pair<const double *, const double *>> covariance_blocks;
  covariance_blocks.emplace_back(orientation_variable->data(),
                                 orientation_variable->data());
  covariance_blocks.emplace_back(orientation_variable->data(),
                                 position_variable->data());
  covariance_blocks.emplace_back(orientation_variable->data(),
                                 velocity_variable->data());
  covariance_blocks.emplace_back(orientation_variable->data(),
                                 gyro_bias_variable->data());
  covariance_blocks.emplace_back(orientation_variable->data(),
                                 accel_bias_variable->data());
  covariance_blocks.emplace_back(position_variable->data(),
                                 position_variable->data());
  covariance_blocks.emplace_back(position_variable->data(),
                                 velocity_variable->data());
  covariance_blocks.emplace_back(position_variable->data(),
                                 gyro_bias_variable->data());
  covariance_blocks.emplace_back(position_variable->data(),
                                 accel_bias_variable->data());
  covariance_blocks.emplace_back(velocity_variable->data(),
                                 velocity_variable->data());
  covariance_blocks.emplace_back(velocity_variable->data(),
                                 gyro_bias_variable->data());
  covariance_blocks.emplace_back(velocity_variable->data(),
                                 accel_bias_variable->data());
  covariance_blocks.emplace_back(gyro_bias_variable->data(),
                                 gyro_bias_variable->data());
  covariance_blocks.emplace_back(gyro_bias_variable->data(),
                                 accel_bias_variable->data());
  covariance_blocks.emplace_back(accel_bias_variable->data(),
                                 accel_bias_variable->data());

  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);
  covariance.Compute(covariance_blocks, &problem);

  fuse_core::MatrixXd cov_or_or(orientation_variable->localSize(),
                                orientation_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(orientation_variable->data(),
                                              orientation_variable->data(),
                                              cov_or_or.data());

  fuse_core::MatrixXd cov_or_pos(orientation_variable->localSize(),
                                 position_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(orientation_variable->data(),
                                              position_variable->data(),
                                              cov_or_pos.data());

  fuse_core::MatrixXd cov_or_vel(orientation_variable->localSize(),
                                 velocity_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(orientation_variable->data(),
                                              velocity_variable->data(),
                                              cov_or_vel.data());

  fuse_core::MatrixXd cov_or_bg(orientation_variable->localSize(),
                                gyro_bias_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(orientation_variable->data(),
                                              gyro_bias_variable->data(),
                                              cov_or_bg.data());

  fuse_core::MatrixXd cov_or_ba(orientation_variable->localSize(),
                                accel_bias_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(orientation_variable->data(),
                                              accel_bias_variable->data(),
                                              cov_or_ba.data());

  fuse_core::MatrixXd cov_pos_pos(position_variable->size(),
                                  position_variable->size());
  covariance.GetCovarianceBlock(position_variable->data(),
                                position_variable->data(), cov_pos_pos.data());

  fuse_core::MatrixXd cov_pos_vel(position_variable->size(),
                                  velocity_variable->size());
  covariance.GetCovarianceBlock(position_variable->data(),
                                velocity_variable->data(), cov_pos_vel.data());

  fuse_core::MatrixXd cov_pos_bg(position_variable->size(),
                                 gyro_bias_variable->size());
  covariance.GetCovarianceBlock(position_variable->data(),
                                gyro_bias_variable->data(), cov_pos_bg.data());

  fuse_core::MatrixXd cov_pos_ba(position_variable->size(),
                                 accel_bias_variable->size());
  covariance.GetCovarianceBlock(position_variable->data(),
                                accel_bias_variable->data(), cov_pos_ba.data());

  fuse_core::MatrixXd cov_vel_vel(velocity_variable->size(),
                                  velocity_variable->size());
  covariance.GetCovarianceBlock(velocity_variable->data(),
                                velocity_variable->data(), cov_vel_vel.data());

  fuse_core::MatrixXd cov_vel_bg(velocity_variable->size(),
                                 gyro_bias_variable->size());
  covariance.GetCovarianceBlock(velocity_variable->data(),
                                gyro_bias_variable->data(), cov_vel_bg.data());

  fuse_core::MatrixXd cov_vel_ba(velocity_variable->size(),
                                 accel_bias_variable->size());
  covariance.GetCovarianceBlock(velocity_variable->data(),
                                accel_bias_variable->data(), cov_vel_ba.data());

  fuse_core::MatrixXd cov_bg_bg(gyro_bias_variable->size(),
                                gyro_bias_variable->size());
  covariance.GetCovarianceBlock(gyro_bias_variable->data(),
                                gyro_bias_variable->data(), cov_bg_bg.data());

  fuse_core::MatrixXd cov_bg_ba(gyro_bias_variable->size(),
                                accel_bias_variable->size());
  covariance.GetCovarianceBlock(gyro_bias_variable->data(),
                                accel_bias_variable->data(), cov_bg_ba.data());

  fuse_core::MatrixXd cov_ba_ba(accel_bias_variable->size(),
                                accel_bias_variable->size());
  covariance.GetCovarianceBlock(accel_bias_variable->data(),
                                accel_bias_variable->data(), cov_ba_ba.data());

  // Assemble the full covariance from the covariance blocks
  Eigen::Matrix<double, 15, 15> actual_covariance;
  actual_covariance << cov_or_or, cov_or_pos, cov_or_vel, cov_or_bg, cov_or_ba,
      cov_or_pos.transpose(), cov_pos_pos, cov_pos_vel, cov_pos_bg, cov_pos_ba,
      cov_or_vel.transpose(), cov_pos_vel.transpose(), cov_vel_vel, cov_vel_bg,
      cov_vel_ba, cov_or_bg.transpose(), cov_pos_bg.transpose(),
      cov_vel_bg.transpose(), cov_bg_bg, cov_bg_ba, cov_or_ba.transpose(),
      cov_pos_ba.transpose(), cov_vel_ba.transpose(), cov_bg_ba.transpose(),
      cov_ba_ba;

  // Define the expected covariance
  Eigen::Matrix<double, 15, 15> expected_covariance{data_.cov};

  EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-5);
}

TEST(AbsoluteImuState3DStampedConstraint, Serialization) {
  // Construct a constraint
  bs_constraints::global::AbsoluteImuState3DStampedConstraint expected(
      "test", data_.imu_state, data_.mean, data_.cov);

  // Serialize the constraint into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new constraint from that same stream
  bs_constraints::global::AbsoluteImuState3DStampedConstraint actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.uuid(), actual.uuid());
  EXPECT_EQ(expected.variables(), actual.variables());
  EXPECT_MATRIX_EQ(expected.mean(), actual.mean());
  EXPECT_MATRIX_EQ(expected.sqrtInformation(), actual.sqrtInformation());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
