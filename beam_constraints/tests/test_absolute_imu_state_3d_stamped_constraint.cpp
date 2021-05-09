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

#include <beam_constraints/global/absolute_imu_state_3d_stamped_constraint.h>

class Data {
public:
  Data() {
    device_id = fuse_core::uuid::generate("r5d4");

    orientation1 = fuse_variables::Orientation3DStamped::make_shared(
        ros::Time(1234, 5678), device_id);
    position1 = fuse_variables::Position3DStamped::make_shared(
        ros::Time(1234, 5678), device_id);        
    velocity1 = fuse_variables::VelocityLinear3DStamped::make_shared(
        ros::Time(1234, 5678), device_id);
    gyrobias1 = beam_variables::ImuBiasStamped::make_shared(
        ros::Time(1234, 5678), device_id);
    accelbias1 = beam_variables::ImuBiasStamped::make_shared(
        ros::Time(1234, 5678), device_id);

    mean << 1.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3,
        0.1, 0.2, 0.3;

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

    absolute_imu_state_constraint = std::make_shared<
        beam_constraints::global::AbsoluteImuState3DStampedConstraint>(
        "test", *orientation1, *position1, *velocity1, *gyrobias1, *accelbias1, 
        mean, cov);
  }

  fuse_variables::Orientation3DStamped::SharedPtr orientation1;
  fuse_variables::Position3DStamped::SharedPtr position1;
  fuse_variables::VelocityLinear3DStamped::SharedPtr velocity1;
  beam_variables::ImuBiasStamped::SharedPtr gyrobias1;
  beam_variables::ImuBiasStamped::SharedPtr accelbias1;

  Eigen::Matrix<double, 16, 1> mean;
  Eigen::Matrix<double, 15, 15> cov;

  beam_constraints::global::AbsoluteImuState3DStampedConstraint::SharedPtr
      absolute_imu_state_constraint;

private:
  fuse_core::UUID device_id;
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
  Optimize a single imu state and single constraint, verify the
  expected value and covariance are generated. Create the variables
  */

  auto orientation_variable = fuse_variables::Orientation3DStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  orientation_variable->w() = 0.952;
  orientation_variable->x() = 0.038;
  orientation_variable->y() = -0.189;
  orientation_variable->z() = 0.239;

  auto position_variable = fuse_variables::Position3DStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  position_variable->x() = 1.5;
  position_variable->y() = -3.0;
  position_variable->z() = 10.0;

  auto velocity_variable = fuse_variables::VelocityLinear3DStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  velocity_variable->x() = 1.5;
  velocity_variable->y() = -3.0;
  velocity_variable->z() = 10.0;

  auto gyroscope_bias_variable = beam_variables::ImuBiasStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  gyroscope_bias_variable->x() = 0.15;
  gyroscope_bias_variable->y() = -0.30;
  gyroscope_bias_variable->z() = 1.0;

  auto acceleration_bias_variable = beam_variables::ImuBiasStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  acceleration_bias_variable->x() = 0.15;
  acceleration_bias_variable->y() = -0.30;
  acceleration_bias_variable->z() = 1.0;

  // Create an absolute pose constraint
  Eigen::Matrix<double, 16, 1> mean;
  mean << 1.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.1,
      0.2, 0.3;

  // clang-format off
  Eigen::Matrix<double, 15, 15> cov;
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

  auto constraint = beam_constraints::global::
      AbsoluteImuState3DStampedConstraint::make_shared(
          "test", *orientation_variable, *position_variable, *velocity_variable,
          *gyroscope_bias_variable, *acceleration_bias_variable, mean, cov);

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
  problem.AddParameterBlock(gyroscope_bias_variable->data(),
                            gyroscope_bias_variable->size(),
                            gyroscope_bias_variable->localParameterization());
  problem.AddParameterBlock(acceleration_bias_variable->data(),
                            acceleration_bias_variable->size(),
                            acceleration_bias_variable->localParameterization());

  std::vector<double *> parameter_blocks;
  parameter_blocks.push_back(orientation_variable->data());
  parameter_blocks.push_back(position_variable->data());  
  parameter_blocks.push_back(velocity_variable->data());
  parameter_blocks.push_back(gyroscope_bias_variable->data());
  parameter_blocks.push_back(acceleration_bias_variable->data());

  problem.AddResidualBlock(constraint->costFunction(),
                           constraint->lossFunction(), parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(1.0, orientation_variable->w(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->x(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->y(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->z(), 1.0e-3);
  EXPECT_NEAR(1.0, position_variable->x(), 1.0e-5);
  EXPECT_NEAR(2.0, position_variable->y(), 1.0e-5);
  EXPECT_NEAR(3.0, position_variable->z(), 1.0e-5);
  EXPECT_NEAR(1.0, velocity_variable->x(), 1.0e-5);
  EXPECT_NEAR(2.0, velocity_variable->y(), 1.0e-5);
  EXPECT_NEAR(3.0, velocity_variable->z(), 1.0e-5);
  EXPECT_NEAR(0.1, gyroscope_bias_variable->x(), 1.0e-5);
  EXPECT_NEAR(0.2, gyroscope_bias_variable->y(), 1.0e-5);
  EXPECT_NEAR(0.3, gyroscope_bias_variable->z(), 1.0e-5);
  EXPECT_NEAR(0.1, acceleration_bias_variable->x(), 1.0e-5);
  EXPECT_NEAR(0.2, acceleration_bias_variable->y(), 1.0e-5);
  EXPECT_NEAR(0.3, acceleration_bias_variable->z(), 1.0e-5);
 
  // Compute the covariance
  std::vector<std::pair<const double *, const double *> > covariance_blocks;
  covariance_blocks.emplace_back(orientation_variable->data(),
                                 orientation_variable->data());
  covariance_blocks.emplace_back(orientation_variable->data(),
                                 position_variable->data());
  covariance_blocks.emplace_back(orientation_variable->data(),
                                 velocity_variable->data());
  covariance_blocks.emplace_back(orientation_variable->data(),
                                 gyroscope_bias_variable->data());
  covariance_blocks.emplace_back(orientation_variable->data(),
                                 acceleration_bias_variable->data());
  covariance_blocks.emplace_back(position_variable->data(),
                                 position_variable->data());
  covariance_blocks.emplace_back(position_variable->data(),
                                 velocity_variable->data());
  covariance_blocks.emplace_back(position_variable->data(),
                                 gyroscope_bias_variable->data());
  covariance_blocks.emplace_back(position_variable->data(),
                                 acceleration_bias_variable->data());
  covariance_blocks.emplace_back(velocity_variable->data(),
                                 velocity_variable->data());
  covariance_blocks.emplace_back(velocity_variable->data(),
                                 gyroscope_bias_variable->data());
  covariance_blocks.emplace_back(velocity_variable->data(),
                                 acceleration_bias_variable->data());
  covariance_blocks.emplace_back(gyroscope_bias_variable->data(),
                                 gyroscope_bias_variable->data());
  covariance_blocks.emplace_back(gyroscope_bias_variable->data(),
                                 acceleration_bias_variable->data());
  covariance_blocks.emplace_back(acceleration_bias_variable->data(),
                                 acceleration_bias_variable->data());

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
                                gyroscope_bias_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(orientation_variable->data(),
                                              gyroscope_bias_variable->data(),
                                              cov_or_bg.data());

  fuse_core::MatrixXd cov_or_ba(orientation_variable->localSize(),
                                acceleration_bias_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(orientation_variable->data(),
                                              acceleration_bias_variable->data(),
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
                                 gyroscope_bias_variable->size());
  covariance.GetCovarianceBlock(position_variable->data(),
                                gyroscope_bias_variable->data(),
                                cov_pos_bg.data());

  fuse_core::MatrixXd cov_pos_ba(position_variable->size(),
                                 acceleration_bias_variable->size());
  covariance.GetCovarianceBlock(position_variable->data(),
                                acceleration_bias_variable->data(),
                                cov_pos_ba.data());

  fuse_core::MatrixXd cov_vel_vel(velocity_variable->size(),
                                  velocity_variable->size());
  covariance.GetCovarianceBlock(velocity_variable->data(),
                                velocity_variable->data(), cov_vel_vel.data());

  fuse_core::MatrixXd cov_vel_bg(velocity_variable->size(),
                                 gyroscope_bias_variable->size());
  covariance.GetCovarianceBlock(velocity_variable->data(),
                                gyroscope_bias_variable->data(),
                                cov_vel_bg.data());

  fuse_core::MatrixXd cov_vel_ba(velocity_variable->size(),
                                 acceleration_bias_variable->size());
  covariance.GetCovarianceBlock(velocity_variable->data(),
                                acceleration_bias_variable->data(),
                                cov_vel_ba.data());

  fuse_core::MatrixXd cov_bg_bg(gyroscope_bias_variable->size(),
                                gyroscope_bias_variable->size());
  covariance.GetCovarianceBlock(gyroscope_bias_variable->data(),
                                gyroscope_bias_variable->data(),
                                cov_bg_bg.data());

  fuse_core::MatrixXd cov_bg_ba(gyroscope_bias_variable->size(),
                                acceleration_bias_variable->size());
  covariance.GetCovarianceBlock(gyroscope_bias_variable->data(),
                                acceleration_bias_variable->data(),
                                cov_bg_ba.data());

  fuse_core::MatrixXd cov_ba_ba(acceleration_bias_variable->size(),
                                acceleration_bias_variable->size());
  covariance.GetCovarianceBlock(acceleration_bias_variable->data(),
                                acceleration_bias_variable->data(),
                                cov_ba_ba.data());

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
  Eigen::Matrix<double, 15, 15> expected_covariance;
  expected_covariance = cov;

  EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-5);
}

TEST(AbsoluteImuState3DStampedConstraint, Serialization) {
  // Construct a constraint
  beam_constraints::global::AbsoluteImuState3DStampedConstraint expected(
      "test", *(data_.orientation1), *(data_.position1), *(data_.velocity1),
      *(data_.gyrobias1), *(data_.accelbias1), data_.mean, data_.cov);

  // Serialize the constraint into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new constraint from that same stream
  beam_constraints::global::AbsoluteImuState3DStampedConstraint actual;
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
