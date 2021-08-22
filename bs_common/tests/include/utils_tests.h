#pragma once

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include <beam_utils/math.h>

namespace bs_common {

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

}  // namespace bs_common