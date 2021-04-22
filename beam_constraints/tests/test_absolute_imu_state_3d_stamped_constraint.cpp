#include <utility>
#include <vector>

#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <fuse_core/eigen.h>
#include <fuse_core/eigen_gtest.h>
#include <fuse_core/serialization.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>
#include <gtest/gtest.h>

#include <beam_constraints/global/absolute_imu_state_3d_stamped_constraint.h>

class Data {
public:
  Data() {
    device_id = fuse_core::uuid::generate("r5d4");

    position1 = fuse_variables::Position3DStamped::make_shared(
        ros::Time(1234, 5678), device_id);
    velocity1 = fuse_variables::VelocityLinear3DStamped::make_shared(
        ros::Time(1234, 5678), device_id);
    orientation1 = fuse_variables::Orientation3DStamped::make_shared(
        ros::Time(1234, 5678), device_id);

    mean << 1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0;

    // clang-format off
    // Generated PD matrix using Octave: R = rand(9, 9); A = R * R' (use format long g to get the required precision)
    cov << 
        1.8474698649, 1.5204239227, 1.8268879911, 1.8894107711, 1.3432517572, 2.1994192029, 2.0933229335, 1.3251861341, 1.5474423658,
        1.5204239227, 2.4090069961, 1.4207341838, 2.2446380502, 1.7666990314, 2.1207949243, 1.8113780006, 0.9729051405, 1.4955587413,
        1.8268879911, 1.4207341838, 2.8052306197, 1.8733498772, 2.3295649715, 2.9025730627, 2.3237542542, 2.1904533316, 2.0867534199,
        1.8894107711, 2.2446380502, 1.8733498772, 4.3630560845, 2.2862820734, 3.3682779435, 3.1578009118, 2.2688251098, 2.2015121310,
        1.3432517572, 1.7666990314, 2.3295649715, 2.2862820734, 2.8856134090, 2.8859797560, 2.3028466058, 2.2609014549, 2.1018661364,
        2.1994192029, 2.1207949243, 2.9025730627, 3.3682779435, 2.8859797560, 3.9448307716, 3.5154568449, 2.8932304837, 2.6603791289,
        2.0933229335, 1.8113780006, 2.3237542542, 3.1578009118, 2.3028466058, 3.5154568449, 3.4984390458, 2.5805662272, 2.2149242549,
        1.3251861341, 0.9729051405, 2.1904533316, 2.2688251098, 2.2609014549, 2.8932304837, 2.5805662272, 2.6451548322, 1.8742007939,
        1.5474423658, 1.4955587413, 2.0867534199, 2.2015121310, 2.1018661364, 2.6603791289, 2.2149242549, 1.8742007939, 2.8698714081;
    // clang-format on

    absolute_pose_with_velocity_constraint = std::make_shared<
        beam_constraints::global::AbsoluteImuState3DStampedConstraint>(
        "test", *position1, *velocity1, *orientation1, mean, cov);
  }

  fuse_variables::Position3DStamped::SharedPtr position1;
  fuse_variables::VelocityLinear3DStamped::SharedPtr velocity1;
  fuse_variables::Orientation3DStamped::SharedPtr orientation1;
  Eigen::Matrix<double, 10, 1> mean;
  fuse_core::Matrix9d cov;

  beam_constraints::global::AbsoluteImuState3DStampedConstraint::SharedPtr
      absolute_pose_with_velocity_constraint;

private:
  fuse_core::UUID device_id;
};

Data data_;

TEST(AbsoluteImuState3DStampedConstraint, Constructor) {
  EXPECT_NO_THROW(*data_.absolute_pose_with_velocity_constraint);
}

TEST(AbsoluteImuState3DStampedConstraint, Covariance) {
  fuse_core::Matrix9d expected_cov = data_.cov;
  fuse_core::Matrix9d expected_sqrt_info = data_.cov.inverse().llt().matrixU();

  auto constraint = *data_.absolute_pose_with_velocity_constraint;

  EXPECT_MATRIX_NEAR(expected_cov, constraint.covariance(), 1.0e-9);
  EXPECT_MATRIX_NEAR(expected_sqrt_info, constraint.sqrtInformation(), 1.0e-9);
}

TEST(AbsoluteImuState3DStampedConstraint, Optimization) {
  /*
  Optimize a single pose with velocity and single constraint, verify the
  expected value and covariance are generated. Create the variables
  */

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

  auto orientation_variable = fuse_variables::Orientation3DStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  orientation_variable->w() = 0.952;
  orientation_variable->x() = 0.038;
  orientation_variable->y() = -0.189;
  orientation_variable->z() = 0.239;

  // Create an absolute pose constraint
  Eigen::Matrix<double, 10, 1> mean;
  mean << 1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0;

  // clang-format off
  fuse_core::Matrix9d cov;
  cov << 1.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8,
         0.1, 2.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3,
         0.2, 0.9, 3.0, 0.2, 0.1, 0.2, 0.3, 0.4, 0.5,
         0.3, 0.8, 0.2, 4.0, 0.3, 0.4, 0.5, 0.6, 0.7,
         0.4, 0.7, 0.1, 0.3, 5.0, 0.5, 0.6, 0.7, 0.8,
         0.5, 0.6, 0.2, 0.4, 0.5, 6.0, 0.6, 0.7, 0.8,
				 0.6, 0.5, 0.3, 0.5, 0.6, 0.6, 7.0, 0.7, 0.8, 
				 0.7, 0.4, 0.4, 0.6, 0.7, 0.7, 0.7, 8.0, 0.7,
				 0.8, 0.3, 0.5, 0.7, 0.8, 0.8, 0.8, 0.7, 9.0;
  // clang-format on

  auto constraint =
      beam_constraints::global::AbsoluteImuState3DStampedConstraint::
          make_shared("test", *position_variable, *velocity_variable,
                      *orientation_variable, mean, cov);

  // Build the problem
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
  ceres::Problem problem(problem_options);

  problem.AddParameterBlock(position_variable->data(),
                            position_variable->size(),
                            position_variable->localParameterization());
  problem.AddParameterBlock(velocity_variable->data(),
                            velocity_variable->size(),
                            velocity_variable->localParameterization());
  problem.AddParameterBlock(orientation_variable->data(),
                            orientation_variable->size(),
                            orientation_variable->localParameterization());

  std::vector<double *> parameter_blocks;
  parameter_blocks.push_back(position_variable->data());
  parameter_blocks.push_back(velocity_variable->data());
  parameter_blocks.push_back(orientation_variable->data());

  problem.AddResidualBlock(constraint->costFunction(),
                           constraint->lossFunction(), parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(1.0, position_variable->x(), 1.0e-5);
  EXPECT_NEAR(2.0, position_variable->y(), 1.0e-5);
  EXPECT_NEAR(3.0, position_variable->z(), 1.0e-5);
  EXPECT_NEAR(1.0, velocity_variable->x(), 1.0e-5);
  EXPECT_NEAR(2.0, velocity_variable->y(), 1.0e-5);
  EXPECT_NEAR(3.0, velocity_variable->z(), 1.0e-5);
  EXPECT_NEAR(1.0, orientation_variable->w(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->x(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->y(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->z(), 1.0e-3);

  // Compute the covariance
  std::vector<std::pair<const double *, const double *> > covariance_blocks;
  covariance_blocks.emplace_back(position_variable->data(),
                                 position_variable->data());
  covariance_blocks.emplace_back(position_variable->data(),
                                 velocity_variable->data());
  covariance_blocks.emplace_back(position_variable->data(),
                                 orientation_variable->data());
  covariance_blocks.emplace_back(velocity_variable->data(),
                                 velocity_variable->data());
  covariance_blocks.emplace_back(velocity_variable->data(),
                                 orientation_variable->data());
  covariance_blocks.emplace_back(orientation_variable->data(),
                                 orientation_variable->data());

  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);
  covariance.Compute(covariance_blocks, &problem);

  fuse_core::MatrixXd cov_pos_pos(position_variable->size(),
                                  position_variable->size());
  covariance.GetCovarianceBlock(position_variable->data(),
                                position_variable->data(), cov_pos_pos.data());

  fuse_core::MatrixXd cov_pos_vel(position_variable->size(),
                                  velocity_variable->size());
  covariance.GetCovarianceBlock(position_variable->data(),
                                velocity_variable->data(), cov_pos_vel.data());

  fuse_core::MatrixXd cov_pos_or(position_variable->localSize(),
                                 orientation_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(position_variable->data(),
                                              orientation_variable->data(),
                                              cov_pos_or.data());

  fuse_core::MatrixXd cov_vel_vel(velocity_variable->size(),
                                  velocity_variable->size());
  covariance.GetCovarianceBlock(velocity_variable->data(),
                                velocity_variable->data(), cov_vel_vel.data());

  fuse_core::MatrixXd cov_vel_or(velocity_variable->localSize(),
                                 orientation_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(velocity_variable->data(),
                                              orientation_variable->data(),
                                              cov_vel_or.data());

  fuse_core::MatrixXd cov_or_or(orientation_variable->localSize(),
                                orientation_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(orientation_variable->data(),
                                              orientation_variable->data(),
                                              cov_or_or.data());

  // Assemble the full covariance from the covariance blocks
  fuse_core::Matrix9d actual_covariance;
  actual_covariance << cov_pos_pos, cov_pos_vel, cov_pos_or,
      cov_pos_vel.transpose(), cov_vel_vel, cov_vel_or, cov_pos_or.transpose(),
      cov_vel_or.transpose(), cov_or_or;

  // Define the expected covariance
  fuse_core::Matrix9d expected_covariance;
  expected_covariance = cov;

  EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-5);
}

TEST(AbsoluteImuState3DStampedConstraint, Serialization) {
  // Construct a constraint
  beam_constraints::global::AbsoluteImuState3DStampedConstraint expected(
      "test", *(data_.position1), *(data_.velocity1), *(data_.orientation1),
      data_.mean, data_.cov);

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
