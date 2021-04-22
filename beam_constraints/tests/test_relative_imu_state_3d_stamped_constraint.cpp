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
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <fuse_variables/velocity_linear_3d_stamped.h>
#include <gtest/gtest.h>

#include <beam_constraints/frame_to_frame/relative_imu_state_3d_stamped_constraint.h>
#include <beam_constraints/global/absolute_constraint.h>

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

    position2 = fuse_variables::Position3DStamped::make_shared(
        ros::Time(1235, 5678), device_id);
    velocity2 = fuse_variables::VelocityLinear3DStamped::make_shared(
        ros::Time(1235, 5678), device_id);
    orientation2 = fuse_variables::Orientation3DStamped::make_shared(
        ros::Time(1235, 5678), device_id);

    delta << 1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 0.988, 0.094, 0.079, 0.094;

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

    relative_pose_with_velocity_constraint = std::make_shared<
        beam_constraints::frame_to_frame::RelativeImuState3DStampedConstraint>(
        "test", *position1, *velocity1, *orientation1, *position2, *velocity2,
        *orientation2, delta, cov);
  }

  fuse_variables::Position3DStamped::SharedPtr position1;
  fuse_variables::VelocityLinear3DStamped::SharedPtr velocity1;
  fuse_variables::Orientation3DStamped::SharedPtr orientation1;
  fuse_variables::Position3DStamped::SharedPtr position2;
  fuse_variables::VelocityLinear3DStamped::SharedPtr velocity2;
  fuse_variables::Orientation3DStamped::SharedPtr orientation2;
  Eigen::Matrix<double, 10, 1> delta;
  fuse_core::Matrix9d cov;

  beam_constraints::frame_to_frame::RelativeImuState3DStampedConstraint::
      SharedPtr relative_pose_with_velocity_constraint;

private:
  fuse_core::UUID device_id;
};

Data data_;

TEST(RelativeImuState3DStampedConstraint, Constructor) {
  EXPECT_NO_THROW(*data_.relative_pose_with_velocity_constraint);
}

TEST(RelativeImuState3DStampedConstraint, Covariance) {
  fuse_core::Matrix9d expected_cov = data_.cov;
  fuse_core::Matrix9d expected_sqrt_info = data_.cov.inverse().llt().matrixU();

  auto constraint = *data_.relative_pose_with_velocity_constraint;

  EXPECT_MATRIX_NEAR(expected_cov, constraint.covariance(), 1.0e-9);
  EXPECT_MATRIX_NEAR(expected_sqrt_info, constraint.sqrtInformation(), 1.0e-9);
}

TEST(RelativeImuState3DStampedConstraint, Optimization) {
  /*
  Optimize a two node system, where:
  1) An absolute pose constraint is generated at the origin of the world frame
  2) An absolute linear velocity constraint is generated at the origin of
  the world frame 3) A relative pose with velocity constraint is generated
  between [P,V,R]_1 and [P,V,R]_2, where: P: Position V: Velocity R: Orientation
  */

  // Create [P,V,R]_1
  auto position1 = fuse_variables::Position3DStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  position1->x() = 1.5;
  position1->y() = -3.0;
  position1->z() = 10.0;

  auto velocity1 = fuse_variables::VelocityLinear3DStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  velocity1->x() = 1.5;
  velocity1->y() = -3.0;
  velocity1->z() = 10.0;

  auto orientation1 = fuse_variables::Orientation3DStamped::make_shared(
      ros::Time(1, 0), fuse_core::uuid::generate("spra"));
  orientation1->w() = 0.952;
  orientation1->x() = 0.038;
  orientation1->y() = -0.189;
  orientation1->z() = 0.239;

  // Create [P,V,R]_2
  auto position2 = fuse_variables::Position3DStamped::make_shared(
      ros::Time(2, 0), fuse_core::uuid::generate("spra"));
  position2->x() = -1.5;
  position2->y() = 3.0;
  position2->z() = -10.0;

  auto velocity2 = fuse_variables::VelocityLinear3DStamped::make_shared(
      ros::Time(2, 0), fuse_core::uuid::generate("spra"));
  velocity2->x() = -1.5;
  velocity2->y() = 3.0;
  velocity2->z() = -10.0;

  auto orientation2 = fuse_variables::Orientation3DStamped::make_shared(
      ros::Time(2, 0), fuse_core::uuid::generate("spra"));
  orientation2->w() = 0.944;
  orientation2->x() = -0.128;
  orientation2->y() = 0.145;
  orientation2->z() = -0.269;

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

  // Create a relative pose with velocity constraint, assuming:
  // 1) a +1m change in position in the x direction
  // 2) a +1/ms change in velocity in the x direction
  Eigen::Matrix<double, 10, 1> mean_delta;
  mean_delta << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  fuse_core::Matrix9d cov_delta = fuse_core::Matrix9d::Identity();
  auto relative_pose_with_velocity =
      beam_constraints::frame_to_frame::RelativeImuState3DStampedConstraint::
          make_shared("test", *position1, *velocity1, *orientation1, *position2,
                      *velocity2, *orientation2, mean_delta, cov_delta);

  // Build the problem
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
  ceres::Problem problem(problem_options);

  problem.AddParameterBlock(position1->data(), position1->size(),
                            position1->localParameterization());
  problem.AddParameterBlock(velocity1->data(), velocity1->size(),
                            velocity1->localParameterization());
  problem.AddParameterBlock(orientation1->data(), orientation1->size(),
                            orientation1->localParameterization());
  problem.AddParameterBlock(position2->data(), position2->size(),
                            position2->localParameterization());
  problem.AddParameterBlock(velocity2->data(), velocity2->size(),
                            velocity2->localParameterization());
  problem.AddParameterBlock(orientation2->data(), orientation2->size(),
                            orientation2->localParameterization());

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

  std::vector<double *> relative_pose_with_velocity_parameter_blocks;
  relative_pose_with_velocity_parameter_blocks.push_back(position1->data());
  relative_pose_with_velocity_parameter_blocks.push_back(velocity1->data());
  relative_pose_with_velocity_parameter_blocks.push_back(orientation1->data());
  relative_pose_with_velocity_parameter_blocks.push_back(position2->data());
  relative_pose_with_velocity_parameter_blocks.push_back(velocity2->data());
  relative_pose_with_velocity_parameter_blocks.push_back(orientation2->data());
  problem.AddResidualBlock(relative_pose_with_velocity->costFunction(),
                           relative_pose_with_velocity->lossFunction(),
                           relative_pose_with_velocity_parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(0.0, position1->x(), 1.0e-5);
  EXPECT_NEAR(0.0, position1->y(), 1.0e-5);
  EXPECT_NEAR(0.0, position1->z(), 1.0e-5);
  EXPECT_NEAR(0.0, velocity1->x(), 1.0e-5);
  EXPECT_NEAR(0.0, velocity1->y(), 1.0e-5);
  EXPECT_NEAR(0.0, velocity1->z(), 1.0e-5);
  EXPECT_NEAR(1.0, orientation1->w(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation1->x(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation1->y(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation1->z(), 1.0e-3);
  EXPECT_NEAR(1.0, position2->x(), 1.0e-5);
  EXPECT_NEAR(0.0, position2->y(), 1.0e-5);
  EXPECT_NEAR(0.0, position2->z(), 1.0e-5);
  EXPECT_NEAR(1.0, velocity2->x(), 1.0e-5);
  EXPECT_NEAR(0.0, velocity2->y(), 1.0e-5);
  EXPECT_NEAR(0.0, velocity2->z(), 1.0e-5);
  EXPECT_NEAR(1.0, orientation2->w(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation2->x(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation2->y(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation2->z(), 1.0e-3);

  // Compute the marginal covariance for [P,V,R]_1
  {
    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    covariance_blocks.emplace_back(position1->data(), position1->data());
    covariance_blocks.emplace_back(position1->data(), velocity1->data());
    covariance_blocks.emplace_back(position1->data(), orientation1->data());
    covariance_blocks.emplace_back(velocity1->data(), velocity1->data());
    covariance_blocks.emplace_back(velocity1->data(), orientation1->data());
    covariance_blocks.emplace_back(orientation1->data(), orientation1->data());

    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    covariance.Compute(covariance_blocks, &problem);

    fuse_core::MatrixXd cov_pos_pos(position1->size(), position1->size());
    covariance.GetCovarianceBlock(position1->data(), position1->data(),
                                  cov_pos_pos.data());

    fuse_core::MatrixXd cov_pos_vel(position1->size(), velocity1->size());
    covariance.GetCovarianceBlock(position1->data(), velocity1->data(),
                                  cov_pos_vel.data());

    fuse_core::MatrixXd cov_pos_or(position1->size(), 3);
    covariance.GetCovarianceBlockInTangentSpace(
        position1->data(), orientation1->data(), cov_pos_or.data());

    fuse_core::MatrixXd cov_vel_vel(velocity1->size(), velocity1->size());
    covariance.GetCovarianceBlock(velocity1->data(), velocity1->data(),
                                  cov_vel_vel.data());

    fuse_core::MatrixXd cov_vel_or(velocity1->size(), 3);
    covariance.GetCovarianceBlockInTangentSpace(
        velocity1->data(), orientation1->data(), cov_vel_or.data());

    fuse_core::MatrixXd cov_or_or(3, 3);
    covariance.GetCovarianceBlockInTangentSpace(
        orientation1->data(), orientation1->data(), cov_or_or.data());

    // Assemble the full covariance from the covariance blocks
    fuse_core::Matrix9d actual_covariance;
    actual_covariance << cov_pos_pos, cov_pos_vel, cov_pos_or,
        cov_pos_vel.transpose(), cov_vel_vel, cov_vel_or,
        cov_pos_or.transpose(), cov_vel_or.transpose(), cov_or_or;

    // Define the expected covariance
    fuse_core::Matrix9d expected_covariance;
    expected_covariance.setIdentity();

    EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-9);
  }

  // Compute the marginal covariance for [P,V,R]_2
  {
    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    covariance_blocks.emplace_back(position2->data(), position2->data());
    covariance_blocks.emplace_back(position2->data(), velocity2->data());
    covariance_blocks.emplace_back(position2->data(), orientation2->data());
    covariance_blocks.emplace_back(velocity2->data(), velocity2->data());
    covariance_blocks.emplace_back(velocity2->data(), orientation2->data());
    covariance_blocks.emplace_back(orientation2->data(), orientation2->data());

    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    covariance.Compute(covariance_blocks, &problem);

    fuse_core::MatrixXd cov_pos_pos(position2->size(), position2->size());
    covariance.GetCovarianceBlock(position2->data(), position2->data(),
                                  cov_pos_pos.data());

    fuse_core::MatrixXd cov_pos_vel(position2->size(), velocity2->size());
    covariance.GetCovarianceBlock(position2->data(), velocity2->data(),
                                  cov_pos_vel.data());

    fuse_core::MatrixXd cov_pos_or(position2->size(), 3);
    covariance.GetCovarianceBlockInTangentSpace(
        position2->data(), orientation2->data(), cov_pos_or.data());

    fuse_core::MatrixXd cov_vel_vel(velocity2->size(), velocity2->size());
    covariance.GetCovarianceBlock(velocity2->data(), velocity2->data(),
                                  cov_vel_vel.data());

    fuse_core::MatrixXd cov_vel_or(velocity2->size(), 3);
    covariance.GetCovarianceBlockInTangentSpace(
        velocity2->data(), orientation2->data(), cov_vel_or.data());

    fuse_core::MatrixXd cov_or_or(3, 3);
    covariance.GetCovarianceBlockInTangentSpace(
        orientation2->data(), orientation2->data(), cov_or_or.data());

    // Assemble the full covariance from the covariance blocks
    fuse_core::Matrix9d actual_covariance;
    actual_covariance << cov_pos_pos, cov_pos_vel, cov_pos_or,
        cov_pos_vel.transpose(), cov_vel_vel, cov_vel_or,
        cov_pos_or.transpose(), cov_vel_or.transpose(), cov_or_or;

    // clang-format off
    // Define the expected covariance 
    fuse_core::Matrix9d expected_covariance;
    expected_covariance << 
        2.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  3.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  1.0,
        0.0,  0.0,  3.0,  0.0,  0.0,  1.0,  0.0, -1.0,  0.0,
        0.0,  0.0,  0.0,  2.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  1.0,  0.0,  0.0,  3.0,  0.0,  0.0,  0.0,  1.0,
        0.0,  0.0,  1.0,  0.0,  0.0,  3.0,  0.0, -1.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  2.0,  0.0,  0.0,
        0.0,  0.0, -1.0,  0.0,  0.0, -1.0,  0.0,  2.0,  0.0,
        0.0,  1.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  2.0;
    // clang-format on

    EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-9);
  }
}

TEST(RelativeImuState3DStampedConstraint, Serialization) {
  // Construct a constraint
  beam_constraints::frame_to_frame::RelativeImuState3DStampedConstraint
      expected("test", *(data_.position1), *(data_.velocity1),
               *(data_.orientation1), *(data_.position2), *(data_.velocity2),
               *(data_.orientation2), data_.delta, data_.cov);

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
