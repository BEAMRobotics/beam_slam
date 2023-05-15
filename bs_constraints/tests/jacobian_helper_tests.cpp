#include <beam_utils/utils.h>
#include <gtest/gtest.h>

#include <bs_common/imu_state.h>
#include <bs_constraints/jacobians.h>

#include <ceres/autodiff_cost_function.h>

constexpr double EPS = 1e-8;
constexpr double THRESHOLD = 1e-6;
constexpr int N = 50;

inline void QuaternionInverse(const double in[4], double out[4]) {
  out[0] = in[0];
  out[1] = -in[1];
  out[2] = -in[2];
  out[3] = -in[3];
}

Eigen::Quaterniond OPlus(const Eigen::Quaterniond& q,
                         const Eigen::Vector3d& pert) {
  double x[4] = {q.w(), q.x(), q.y(), q.z()};
  double delta[3] = {pert[0], pert[1], pert[2]};
  double q_delta[4];
  double x_plus_delta[4];
  ceres::AngleAxisToQuaternion(delta, q_delta);
  ceres::QuaternionProduct(x, q_delta, x_plus_delta);
  Eigen::Quaterniond q_pert(x_plus_delta[0], x_plus_delta[1], x_plus_delta[2],
                            x_plus_delta[3]);
  return q_pert;
}

Eigen::Vector3d OMinus(const Eigen::Quaterniond& q1,
                       const Eigen::Quaterniond& q2) {
  double x1[4] = {q1.w(), q1.x(), q1.y(), q1.z()};
  double x2[4] = {q2.w(), q2.x(), q2.y(), q2.z()};
  double delta[3];

  double x1_inverse[4];
  QuaternionInverse(x1, x1_inverse);
  double q_delta[4];
  ceres::QuaternionProduct(x1_inverse, x2, q_delta);
  ceres::QuaternionToAngleAxis(q_delta, delta);
  Eigen::Vector3d delta_out(delta[0], delta[1], delta[2]);
  return delta_out;
}

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
      const auto res_pert = point_rotation(OPlus(q, pert), point_rand);
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
      const auto res_pert = invert_rotation(OPlus(q, pert));
      const auto finite_diff = OMinus(res, res_pert) / EPS;
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
      const auto res_pert = multiply_rotations(OPlus(q_left, pert), q_right);
      const auto finite_diff = OMinus(res, res_pert) / EPS;
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
      const auto res_pert = multiply_rotations(q_left, OPlus(q_right, pert));
      const auto finite_diff = OMinus(res, res_pert) / EPS;
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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
