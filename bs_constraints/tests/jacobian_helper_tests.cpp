#pragma once

#include <beam_utils/utils.h>
#include <gtest/gtest.h>

#include <bs_common/imu_state.h>
#include <bs_constraints/jacobians.h>

constexpr double EPS = 1e-8;
constexpr double THRESHOLD = 1e-6;
constexpr int N = 1;

using namespace bs_constraints;

TEST(DPointRotationDRotation, validity) {
  // create lambda for function to test
  auto point_rotation = [&](const auto& q, const auto& point) {
    const auto result = (q * point);
    return result;
  };
  for (int i = 0; i < N; i++) {
    // random point
    Eigen::Vector3d point_rand =
        beam::UniformRandomVector<3>(0.0, 1.0).normalized();

    // random pose
    Eigen::Matrix4d T_rand = beam::GenerateRandomPose(1.0, 10.0);
    Eigen::Quaterniond q(T_rand.block<3, 3>(0, 0));

    // compute analytical jacobian
    const auto J_analytical = bs_constraints::DPointRotationDRotation(
        q.toRotationMatrix(), point_rand);

    // calculate numerical jacobian
    Eigen::Matrix<double, 3, 3> J_numerical;
    const auto res = point_rotation(q, point_rand);
    for (int i = 0; i < 3; i++) {
      Eigen::Vector3d pert = Eigen::Vector3d::Zero();
      pert[i] = EPS;
      const auto res_pert = point_rotation(SO3BoxPlus(q, pert), point_rand);
      const auto finite_diff = (res_pert - res) / EPS;
      J_numerical.col(i) = finite_diff.transpose();
    }
    EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
  }
}

TEST(DPointRotationDPoint, validity) {
  // create lambda for function to test
  auto point_rotation = [&](const auto& q, const auto& point) {
    const auto result = (q * point);
    return result;
  };
  for (int i = 0; i < N; i++) {
    // random point
    Eigen::Vector3d point_rand =
        beam::UniformRandomVector<3>(0.0, 1.0).normalized();

    // random pose
    Eigen::Matrix4d T_rand = beam::GenerateRandomPose(1.0, 10.0);
    Eigen::Quaterniond q(T_rand.block<3, 3>(0, 0));

    // compute analytical jacobian
    const auto J_analytical =
        bs_constraints::DPointRotationDPoint(q.toRotationMatrix(), point_rand);

    // calculate numerical jacobian
    Eigen::Matrix<double, 3, 3> J_numerical;
    const auto res = point_rotation(q, point_rand);
    for (int i = 0; i < 3; i++) {
      Eigen::Vector3d pert = Eigen::Vector3d::Zero();
      pert[i] = EPS;
      const auto res_pert = point_rotation(q, point_rand + pert);
      const auto finite_diff = (res_pert - res) / EPS;
      J_numerical.col(i) = finite_diff.transpose();
    }
    EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
  }
}

TEST(DInverseRotationDRotation, validity) {
  // create lambda for function to test
  auto invert_rotation = [&](const auto& q) { return q.inverse(); };

  for (int i = 0; i < N; i++) {
    // random pose
    Eigen::Matrix4d T_rand = beam::GenerateRandomPose(1.0, 10.0);
    Eigen::Matrix3d R = T_rand.block<3, 3>(0, 0);
    Eigen::Quaterniond q(R);

    // compute analytical jacobian
    const auto J_analytical = bs_constraints::DInverseRotationDRotation(R);

    // calculate numerical jacobian
    Eigen::Matrix<double, 3, 3> J_numerical;
    const auto res = invert_rotation(q);
    for (int i = 0; i < 3; i++) {
      Eigen::Vector3d pert = Eigen::Vector3d::Zero();
      pert[i] = EPS;
      const auto res_pert = invert_rotation(SO3BoxPlus(q, pert));
      const auto finite_diff = SO3BoxMinus(res, res_pert) / EPS;
      J_numerical.col(i) = finite_diff.transpose();
    }
    EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
  }
}

TEST(DRotationCompositionDLeftRotation, validity) {
  // create lambda for function to test
  auto multiply_rotations = [&](const auto& q_left, const auto& q_right) {
    auto composition = q_left * q_right;
    return composition;
  };

  for (int i = 0; i < N; i++) {
    // random pose
    Eigen::Quaterniond q_left = beam::RandomQuaternion();
    Eigen::Matrix3d R_left(q_left);

    Eigen::Quaterniond q_right = beam::RandomQuaternion();
    Eigen::Matrix3d R_right(q_right);

    // compute analytical jacobian
    const auto J_analytical =
        bs_constraints::DRotationCompositionDLeftRotation(R_left, R_right);

    // calculate numerical jacobian
    Eigen::Matrix<double, 3, 3> J_numerical;
    const auto res = multiply_rotations(q_left, q_right);
    for (int i = 0; i < 3; i++) {
      Eigen::Vector3d pert = Eigen::Vector3d::Zero();
      pert[i] = EPS;
      const auto res_pert =
          multiply_rotations(SO3BoxPlus(q_left, pert), q_right);
      const auto finite_diff = SO3BoxMinus(res, res_pert) / EPS;

      J_numerical.col(i) = finite_diff.transpose();
    }
    EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
  }
}

TEST(DRotationCompositionDRightRotation, validity) {
  // create lambda for function to test
  auto multiply_rotations = [&](const auto& q_left, const auto& q_right) {
    auto composition = q_left * q_right;
    return composition;
  };

  for (int i = 0; i < N; i++) {
    // random pose
    Eigen::Quaterniond q_left = beam::RandomQuaternion();
    Eigen::Matrix3d R_left(q_left);

    Eigen::Quaterniond q_right = beam::RandomQuaternion();
    Eigen::Matrix3d R_right(q_right);

    // compute analytical jacobian
    const auto J_analytical =
        bs_constraints::DRotationCompositionDRightRotation(R_left, R_right);

    // calculate numerical jacobian
    Eigen::Matrix<double, 3, 3> J_numerical;
    const auto res = multiply_rotations(q_left, q_right);
    for (int i = 0; i < 3; i++) {
      Eigen::Vector3d pert = Eigen::Vector3d::Zero();
      pert[i] = EPS;
      const auto res_pert =
          multiply_rotations(q_left, SO3BoxPlus(q_right, pert));
      const auto finite_diff = SO3BoxMinus(res, res_pert) / EPS;
      J_numerical.col(i) = finite_diff.transpose();
    }
    EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
  }
}

TEST(DImageProjectionDPoint, Validity) {
  // create lambda for function to test
  auto projection = [&](const auto& K, const auto& point) {
    return (K * point).hnormalized();
  };

  for (int i = 0; i < N; i++) {
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
      const auto proj = projection(K_rand, point_camera_rand);
      const auto proj_pert = projection(K_rand, point_camera_rand + pert);
      const auto finite_diff = (proj_pert - proj) / EPS;
      J_numerical.col(i) = finite_diff.transpose();
    }
    EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
  }
}

TEST(DPointTransformationDTransform, seperate_parameterizations) {
  // create lambda for function to test
  auto point_transformation = [&](const auto& _T, const auto& _point) {
    const auto result = (_T * _point.homogeneous()).hnormalized();
    return result;
  };
  for (int i = 0; i < N; i++) {
    // random point
    Eigen::Vector3d point = beam::UniformRandomVector<3>(0.0, 1.0).normalized();

    // random pose
    Eigen::Matrix4d T = beam::GenerateRandomPose(1.0, 10.0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Quaterniond q(R);
    Eigen::Matrix<double, 7, 1> T_vec;
    T_vec << t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z();

    // compute analytical jacobian
    Eigen::Matrix<double, 3, 6> J_analytical;
    J_analytical.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    J_analytical.block<3, 3>(0, 3) =
        bs_constraints::DPointRotationDRotation(R, point);

    // calculate numerical jacobian
    Eigen::Matrix<double, 3, 6> J_numerical;
    const auto res = point_transformation(T, point);
    for (int i = 0; i < 6; i++) {
      Eigen::Matrix<double, 6, 1> pert = Eigen::Matrix<double, 6, 1>::Zero();
      pert[i] = EPS;
      auto T_vec_pert = TranslationSO3BoxPlus(T_vec, pert);
      Eigen::Quaterniond q_pert(T_vec_pert[3], T_vec_pert[4], T_vec_pert[5],
                                T_vec_pert[6]);
      Eigen::Vector3d t_pert(T_vec_pert[0], T_vec_pert[1], T_vec_pert[2]);
      Eigen::Matrix4d T_pert = Eigen::Matrix4d::Identity();
      T_pert.block<3, 3>(0, 0) = q_pert.toRotationMatrix();
      T_pert.block<3, 1>(0, 3) = t_pert;

      const auto res_pert = point_transformation(T_pert, point);
      const auto finite_diff = (res_pert - res) / EPS;
      J_numerical.col(i) = finite_diff.transpose();
    }
    EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
  }
}

TEST(DPointTransformationDTransform, single_parameterization) {
  // create lambda for function to test
  auto point_transformation = [&](const auto& _T, const auto& _point) {
    const auto result = (_T * _point.homogeneous()).hnormalized();
    return result;
  };
  for (int i = 0; i < N; i++) {
    // random point
    Eigen::Vector3d point = beam::UniformRandomVector<3>(0.0, 1.0).normalized();

    // random pose
    Eigen::Matrix4d T = beam::GenerateRandomPose(1.0, 10.0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Quaterniond q(R);
    Eigen::Matrix<double, 7, 1> T_vec;
    T_vec << t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z();

    // compute analytical jacobian
    const auto J_analytical =
        bs_constraints::DPointTransformationDTransform(T, point);

    // calculate numerical jacobian
    Eigen::Matrix<double, 3, 6> J_numerical;
    const auto res = point_transformation(T, point);
    for (int i = 0; i < 6; i++) {
      Eigen::Matrix<double, 6, 1> pert = Eigen::Matrix<double, 6, 1>::Zero();
      pert[i] = EPS;
      auto T_vec_pert = SE3BoxPlus(T_vec, pert);
      Eigen::Quaterniond q_pert(T_vec_pert[3], T_vec_pert[4], T_vec_pert[5],
                                T_vec_pert[6]);
      Eigen::Vector3d t_pert(T_vec_pert[0], T_vec_pert[1], T_vec_pert[2]);
      Eigen::Matrix4d T_pert = Eigen::Matrix4d::Identity();
      T_pert.block<3, 3>(0, 0) = q_pert.toRotationMatrix();
      T_pert.block<3, 1>(0, 3) = t_pert;

      const auto res_pert = point_transformation(T_pert, point);
      const auto finite_diff = (res_pert - res) / EPS;
      J_numerical.col(i) = finite_diff.transpose();
    }
    EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
  }
}

TEST(DPointInverseTransformationDRotation, minimal_parameterization) {
  // create lambda for function to test
  auto point_transformation = [&](const auto& _q, const auto& _t,
                                  const auto& _point) {
    const Eigen::Vector3d result =
        (_q.inverse() * _point) - (_q.inverse() * _t);

    return result;
  };
  for (int i = 0; i < N; i++) {
    // random point
    Eigen::Vector3d point = beam::UniformRandomVector<3>(0.0, 1.0).normalized();

    // random pose
    Eigen::Matrix4d T = beam::GenerateRandomPose(1.0, 10.0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Quaterniond q(R);

    // compute analytical jacobian
    const auto J_analytical = DPointRotationDRotation(R.transpose(), point) *
                                  DInverseRotationDRotation(R) -
                              DPointRotationDRotation(R.transpose(), t) *
                                  DInverseRotationDRotation(R);

    // calculate numerical jacobian
    Eigen::Matrix<double, 3, 3> J_numerical;
    const auto res = point_transformation(q, t, point);
    for (int i = 0; i < 3; i++) {
      Eigen::Vector3d pert = Eigen::Vector3d::Zero();
      pert[i] = EPS;
      const auto q_pert = SO3BoxPlus(q, pert);
      const auto res_pert = point_transformation(q_pert, t, point);

      const auto finite_diff = (res_pert - res) / EPS;
      J_numerical.col(i) = finite_diff.transpose();
    }
    EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
  }
}

TEST(DPointInverseTransformationDRotation, naive_parameterization) {
  // create lambda for function to test
  auto point_transformation = [&](const auto& _q, const auto& _t,
                                  const auto& _point) {
    const Eigen::Vector3d result =
        (_q.inverse() * _point) - (_q.inverse() * _t);

    return result;
  };
  for (int i = 0; i < N; i++) {
    // random point
    Eigen::Vector3d point = beam::UniformRandomVector<3>(0.0, 1.0).normalized();

    // random pose
    Eigen::Matrix4d T = beam::GenerateRandomPose(1.0, 10.0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Quaterniond q(R);

    // compute analytical jacobian
    const auto J_minimal = DPointRotationDRotation(R.transpose(), point) *
                               DInverseRotationDRotation(R) -
                           DPointRotationDRotation(R.transpose(), t) *
                               DInverseRotationDRotation(R);
    const auto J_analytical = J_minimal * MinusJacobian(q);

    // calculate numerical jacobian
    Eigen::Matrix<double, 3, 4> J_numerical;
    const auto res = point_transformation(q, t, point);
    for (int i = 0; i < 4; i++) {
      Eigen::Vector4d pert = Eigen::Vector4d::Zero();
      pert[i] = EPS;
      const auto q_pert_coeffs = q.coeffs() + pert;
      const Eigen::Quaterniond q_pert(q_pert_coeffs[3], q_pert_coeffs[0],
                                      q_pert_coeffs[1], q_pert_coeffs[2]);
      const auto res_pert = point_transformation(q_pert, t, point);
      const auto finite_diff = (res_pert - res) / EPS;
      J_numerical.col(i) = finite_diff.transpose();
    }
    std::cout << J_analytical << std::endl;
    std::cout << "-----" << std::endl;
    std::cout << J_numerical << std::endl;

    EXPECT_TRUE(J_numerical.isApprox(J_analytical, THRESHOLD));
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
