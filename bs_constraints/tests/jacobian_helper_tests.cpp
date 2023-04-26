#include <beam_utils/utils.h>
#include <gtest/gtest.h>

#include <bs_common/imu_state.h>
#include <bs_constraints/jacobians.h>

constexpr double EPS = 1e-6;
constexpr double THRESHOLD = 1e-6;
constexpr int N = 50;

TEST(DImageProjectionDPoint, Validity) {
  // create lambda for function to test
  auto projection = [&](const auto& K, const auto& point) {
    return (K * point).hnormalized();
  };

  // generate random 3d point
  Eigen::Vector3d point_camera_rand =
      beam::UniformRandomVector<3>(1.0, 10.0).normalized();
  // generate random intrinsic matrix
  Eigen::Vector2d camera_center = beam::UniformRandomVector<2>(50.0, 100.0);
  double f = beam::randf(10.0, 100.0);
  Eigen::Matrix3d K_rand = Eigen::Matrix3d::Identity();
  K_rand(0, 0) = f;
  K_rand(1, 1) = f;
  K_rand(0, 2) = camera_center.x();
  K_rand(1, 2) = camera_center.y();

  // compute analytical jacobian
  const auto J_analytical =
      bs_constraints::DImageProjectionDPoint(K_rand, point_camera_rand);

  // calculate numerical jacobian
  Eigen::Matrix<double, 2, 3> J_numerical;
  for (int i = 0; i < 3; i++) {
    Eigen::Vector3d pert(0, 0, 0);
    pert[i] = EPS;
    const auto proj = projection(K_rand, point_camera_rand + pert);
    const auto proj_pert = projection(K_rand, point_camera_rand);
    J_numerical(0, i) = (proj_pert[0] - proj[0]) / EPS;
    J_numerical(1, i) = (proj_pert[1] - proj[1]) / EPS;
  }

  EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
}

TEST(DPointTransformationDTransform, Validity) {
  // create lambda for function to test
  auto point_transform = [&](const auto& T, const auto& point) {
    return (T * point.homogeneous()).hnormalized();
  };
  // random point
  Eigen::Vector3d point_rand =
      beam::UniformRandomVector<3>(0.0, 1.0).normalized();

  // random pose
  Eigen::Matrix4d T_rand = beam::GenerateRandomPose(1.0, 10.0);

  // compute analytical jacobian
  const auto J_analytical =
      bs_constraints::DPointTransformationDTransform(T_rand, point_rand);

  // calculate numerical jacobian
  Eigen::Matrix<double, 3, 6> J_numerical;
  for (int i = 0; i < 6; i++) {
    Eigen::Matrix<double, 6, 1> pert = Eigen::Matrix<double, 6, 1>::Zero();
    pert[i] = EPS;
    const auto res = point_transform(T_rand, point_rand);
    const auto res_pert =
        point_transform(beam::BoxPlus(T_rand, pert), point_rand);
    J_numerical(0, i) = (res_pert[0] - res[0]) / EPS;
    J_numerical(1, i) = (res_pert[1] - res[1]) / EPS;
    J_numerical(2, i) = (res_pert[2] - res[2]) / EPS;
  }

  EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
}

TEST(DPointTransformationDPoint, Validity) {
  // create lambda for function to test
  auto point_transform = [&](const auto& T, const auto& point) {
    return (T * point.homogeneous()).hnormalized();
  };

  // random point
  Eigen::Vector3d point_rand =
      beam::UniformRandomVector<3>(0.0, 1.0).normalized();

  // random pose
  Eigen::Matrix4d T_rand = beam::GenerateRandomPose(1.0, 10.0);

  // compute analytical jacobian
  const auto J_analytical =
      bs_constraints::DPointTransformationDTransform(T_rand, point_rand);

  // calculate numerical jacobian
  Eigen::Matrix<double, 3, 6> J_numerical;
  for (int i = 0; i < 6; i++) {
    Eigen::Vector3d pert(0, 0, 0);
    pert[i] = EPS;
    const auto res = point_transform(T_rand, point_rand + pert);
    const auto res_pert = point_transform(T_rand, point_rand);
    J_numerical(0, i) = (res_pert[0] - res[0]) / EPS;
    J_numerical(1, i) = (res_pert[1] - res[1]) / EPS;
    J_numerical(2, i) = (res_pert[2] - res[2]) / EPS;
  }

  EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
}

TEST(DInverseTransformDTransform, Validity) {
  // create lambda for function to test
  auto inv_transform = [&](const auto& T) {
    return beam::TransformToTwistVector(beam::InvertTransform(T));
  };

  // random pose
  Eigen::Matrix4d T_rand = beam::GenerateRandomPose(1.0, 10.0);

  // compute analytical jacobian
  const auto J_analytical = bs_constraints::DInverseTransformDTransform(T_rand);

  // calculate numerical jacobian
  Eigen::Matrix<double, 6, 6> J_numerical;
  for (int i = 0; i < 6; i++) {
    Eigen::Matrix<double, 6, 1> pert = Eigen::Matrix<double, 6, 1>::Zero();
    pert[i] = EPS;
    const auto res = inv_transform(T_rand);
    const auto res_pert = inv_transform(beam::BoxPlus(T_rand, pert));
    J_numerical(0, i) = (res_pert[0] - res[0]) / EPS;
    J_numerical(1, i) = (res_pert[1] - res[1]) / EPS;
    J_numerical(2, i) = (res_pert[2] - res[2]) / EPS;
    J_numerical(3, i) = (res_pert[3] - res[3]) / EPS;
    J_numerical(4, i) = (res_pert[4] - res[4]) / EPS;
    J_numerical(5, i) = (res_pert[5] - res[5]) / EPS;
  }

  EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
}

TEST(DTransformCompositionDRightTransform, Validity) {
  // create lambda for function to test
  auto transform_composition = [&](const auto& T1, const auto& T2) {
    return beam::TransformToTwistVector(T1 * T2);
  };

  // random pose
  Eigen::Matrix4d T1_rand = beam::GenerateRandomPose(1.0, 10.0);
  Eigen::Matrix4d T2_rand = beam::GenerateRandomPose(1.0, 10.0);

  // compute analytical jacobian
  const auto J_analytical =
      bs_constraints::DTransformCompositionDRightTransform();

  // calculate numerical jacobian
  Eigen::Matrix<double, 6, 6> J_numerical;
  for (int i = 0; i < 6; i++) {
    Eigen::Matrix<double, 6, 1> pert = Eigen::Matrix<double, 6, 1>::Zero();
    pert[i] = EPS;
    const auto res = transform_composition(T1_rand, T2_rand);
    const auto res_pert =
        transform_composition(T1_rand, beam::BoxPlus(T2_rand, pert));
    J_numerical(0, i) = (res_pert[0] - res[0]) / EPS;
    J_numerical(1, i) = (res_pert[1] - res[1]) / EPS;
    J_numerical(2, i) = (res_pert[2] - res[2]) / EPS;
    J_numerical(3, i) = (res_pert[3] - res[3]) / EPS;
    J_numerical(4, i) = (res_pert[4] - res[4]) / EPS;
    J_numerical(5, i) = (res_pert[5] - res[5]) / EPS;
  }

  EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
}

TEST(DTransformCompositionDLeftTransform, Validity) {
  // create lambda for function to test
  auto transform_composition = [&](const auto& T1, const auto& T2) {
    return beam::TransformToTwistVector(T1 * T2);
  };

  // random pose
  Eigen::Matrix4d T1_rand = beam::GenerateRandomPose(1.0, 10.0);
  Eigen::Matrix4d T2_rand = beam::GenerateRandomPose(1.0, 10.0);

  // compute analytical jacobian
  const auto J_analytical =
      bs_constraints::DTransformCompositionDLeftTransform(T1_rand);

  // calculate numerical jacobian
  Eigen::Matrix<double, 6, 6> J_numerical;
  for (int i = 0; i < 6; i++) {
    Eigen::Matrix<double, 6, 1> pert = Eigen::Matrix<double, 6, 1>::Zero();
    pert[i] = EPS;
    const auto res = transform_composition(T1_rand, T2_rand);
    const auto res_pert =
        transform_composition(beam::BoxPlus(T1_rand, pert), T2_rand);
    J_numerical(0, i) = (res_pert[0] - res[0]) / EPS;
    J_numerical(1, i) = (res_pert[1] - res[1]) / EPS;
    J_numerical(2, i) = (res_pert[2] - res[2]) / EPS;
    J_numerical(3, i) = (res_pert[3] - res[3]) / EPS;
    J_numerical(4, i) = (res_pert[4] - res[4]) / EPS;
    J_numerical(5, i) = (res_pert[5] - res[5]) / EPS;
  }

  EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
