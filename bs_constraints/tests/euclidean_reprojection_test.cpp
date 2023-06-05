#pragma once

#include <beam_utils/utils.h>
#include <ceres/ceres.h>
#include <gtest/gtest.h>

#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/point_3d_landmark.h>
#include <fuse_variables/position_3d_stamped.h>

#include <bs_common/imu_state.h>
#include <bs_constraints/jacobians.h>
#include <bs_constraints/visual/euclidean_reprojection_function.h>
#include <bs_constraints/visual/reprojection_functor.h>

constexpr double EPS = 1e-8;
constexpr double THRESHOLD = 1e-6;
constexpr int N = 50;

Eigen::Matrix3d GenerateRandomIntrinsicMatrix() {
  Eigen::Vector2d camera_center = beam::UniformRandomVector<2>(100.0, 1000.0);
  double f = beam::randf(10.0, 100.0);
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = f;
  K(1, 1) = f;
  K(0, 2) = camera_center.x();
  K(1, 2) = camera_center.y();
  return K;
}

TEST(EuclideanReprojectionFunction, Validity) {
  // generate random T_WORLD_BASELINK
  const Eigen::Matrix4d T_WORLD_BASELINK = beam::GenerateRandomPose(1.0, 10.0);
  const Eigen::Quaterniond q_WORLD_BASELINK(T_WORLD_BASELINK.block<3, 3>(0, 0));
  const Eigen::Vector3d t_WORLD_BASELINK =
      T_WORLD_BASELINK.block<3, 1>(0, 3).transpose();

  // generate random K
  Eigen::Matrix3d K = GenerateRandomIntrinsicMatrix();

  // generate random T_CAM_BASELINK
  const Eigen::Matrix4d T_CAM_BASELINK = beam::GenerateRandomPose(0.0, 1.0);

  // generate random P_WORLD
  const Eigen::Vector3d P_CAM =
      beam::randf(5.0, 10.0) *
      beam::UniformRandomVector<3>(0.1, 1.0).normalized();
  const Eigen::Vector3d P_WORLD =
      (T_WORLD_BASELINK * beam::InvertTransform(T_CAM_BASELINK) *
       P_CAM.homogeneous())
          .hnormalized();

  const Eigen::Vector2d pixel =
      (K * P_CAM).hnormalized() + Eigen::Vector2d(10, 10);

  // create fuse variables
  ros::Time stamp(0.0);
  uint64_t id = 0;

  // create manual diff cost function
  auto manual_diff_cost_function = std::shared_ptr<ceres::CostFunction>(
      new bs_constraints::EuclideanReprojection(Eigen::Matrix2d::Identity(),
                                                pixel, K, T_CAM_BASELINK));

  // create autodiff cost function
  auto auto_diff_cost_function = std::shared_ptr<ceres::CostFunction>(
      new ceres::AutoDiffCostFunction<bs_constraints::ReprojectionFunctor, 2, 4,
                                      3, 3>(
          new bs_constraints::ReprojectionFunctor(Eigen::Matrix2d::Identity(),
                                                  pixel, K, T_CAM_BASELINK)));

  std::vector<int> manual_rows, auto_rows;
  std::vector<int> manual_cols, auto_cols;
  std::vector<double> manual_vals, auto_vals;
  {
    // Build the problem
    ceres::Problem::Options problem_options;
    problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(problem_options);

    fuse_variables::Orientation3DStamped::SharedPtr orientation =
        fuse_variables::Orientation3DStamped::make_shared(stamp);
    orientation->w() = q_WORLD_BASELINK.w();
    orientation->x() = q_WORLD_BASELINK.x();
    orientation->y() = q_WORLD_BASELINK.y();
    orientation->z() = q_WORLD_BASELINK.z();

    fuse_variables::Position3DStamped::SharedPtr position =
        fuse_variables::Position3DStamped::make_shared(stamp);
    position->x() = t_WORLD_BASELINK[0];
    position->y() = t_WORLD_BASELINK[1];
    position->z() = t_WORLD_BASELINK[2];

    fuse_variables::Point3DLandmark::SharedPtr landmark =
        fuse_variables::Point3DLandmark::make_shared(id);
    landmark->x() = P_WORLD[0];
    landmark->y() = P_WORLD[1];
    landmark->z() = P_WORLD[2];

    problem.AddParameterBlock(orientation->data(), orientation->size(),
                              orientation->localParameterization());
    problem.AddParameterBlock(position->data(), position->size(),
                              position->localParameterization());
    problem.AddParameterBlock(landmark->data(), landmark->size(),
                              landmark->localParameterization());

    std::vector<double*> parameter_blocks;
    parameter_blocks.push_back(orientation->data());
    parameter_blocks.push_back(position->data());
    parameter_blocks.push_back(landmark->data());

    auto loss_function =
        std::shared_ptr<ceres::LossFunction>(new ceres::TrivialLoss());

    problem.AddResidualBlock(manual_diff_cost_function.get(),
                             loss_function.get(), parameter_blocks);

    double cost = 0.0;
    ceres::CRSMatrix jacobian;
    problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, nullptr, nullptr,
                     &jacobian);

    for (auto v : jacobian.values) { manual_vals.push_back(v); }
    for (auto c : jacobian.cols) { manual_cols.push_back(c); }
    for (auto r : jacobian.rows) { manual_rows.push_back(r); }
  }

  {
    // Build the problem
    ceres::Problem::Options problem_options;
    problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(problem_options);

    fuse_variables::Orientation3DStamped::SharedPtr orientation =
        fuse_variables::Orientation3DStamped::make_shared(stamp);
    orientation->w() = q_WORLD_BASELINK.w();
    orientation->x() = q_WORLD_BASELINK.x();
    orientation->y() = q_WORLD_BASELINK.y();
    orientation->z() = q_WORLD_BASELINK.z();

    fuse_variables::Position3DStamped::SharedPtr position =
        fuse_variables::Position3DStamped::make_shared(stamp);
    position->x() = t_WORLD_BASELINK[0];
    position->y() = t_WORLD_BASELINK[1];
    position->z() = t_WORLD_BASELINK[2];

    fuse_variables::Point3DLandmark::SharedPtr landmark =
        fuse_variables::Point3DLandmark::make_shared(id);
    landmark->x() = P_WORLD[0];
    landmark->y() = P_WORLD[1];
    landmark->z() = P_WORLD[2];

    problem.AddParameterBlock(orientation->data(), orientation->size(),
                              orientation->localParameterization());
    problem.AddParameterBlock(position->data(), position->size(),
                              position->localParameterization());
    problem.AddParameterBlock(landmark->data(), landmark->size(),
                              landmark->localParameterization());

    std::vector<double*> parameter_blocks;
    parameter_blocks.push_back(orientation->data());
    parameter_blocks.push_back(position->data());
    parameter_blocks.push_back(landmark->data());

    auto loss_function =
        std::shared_ptr<ceres::LossFunction>(new ceres::TrivialLoss());

    problem.AddResidualBlock(auto_diff_cost_function.get(), loss_function.get(),
                             parameter_blocks);

    double cost = 0.0;
    ceres::CRSMatrix jacobian;
    problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, nullptr, nullptr,
                     &jacobian);

    for (auto v : jacobian.values) { auto_vals.push_back(v); }
    for (auto c : jacobian.cols) { auto_cols.push_back(c); }
    for (auto r : jacobian.rows) { auto_rows.push_back(r); }
  }

  EXPECT_TRUE(auto_vals.size() == manual_vals.size());
  for (int i = 0; i < auto_vals.size(); i++) {
    EXPECT_TRUE(std::abs(auto_vals[i] - manual_vals[i]) < 0.00001);
  }

  EXPECT_TRUE(auto_cols.size() == manual_cols.size());
  for (int i = 0; i < auto_cols.size(); i++) {
    EXPECT_TRUE(auto_cols[i] == manual_cols[i]);
  }

  EXPECT_TRUE(auto_rows.size() == manual_rows.size());
  for (int i = 0; i < auto_rows.size(); i++) {
    EXPECT_TRUE(auto_rows[i] == manual_rows[i]);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
