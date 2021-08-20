#include <gtest/gtest.h>

#include <bs_common/utils.h>
#include <bs_common/imu_state.h>

TEST(ImuPreintegration, ImuState) {
  // create arbitrary state values
  Eigen::Quaterniond q_quat{Eigen::Quaterniond::UnitRandom()};
  Eigen::Vector3d p_vec{1, 2, 3};
  Eigen::Vector3d v_vec{0.1, 0.2, 0.3};
  Eigen::Vector3d bg_vec{0.001, 0.002, 0.003};
  Eigen::Vector3d ba_vec{0.0001, 0.0002, 0.0003};

  // instantiate class
  bs_common::ImuState IS1(ros::Time(0), q_quat, p_vec, v_vec, bg_vec, ba_vec);

  // check fuse/beam variables getters
  EXPECT_EQ(IS1.Stamp(), ros::Time(0));
  EXPECT_EQ(IS1.Orientation().data()[0], q_quat.w());
  EXPECT_EQ(IS1.Orientation().data()[1], q_quat.x());
  EXPECT_EQ(IS1.Orientation().data()[2], q_quat.y());
  EXPECT_EQ(IS1.Orientation().data()[3], q_quat.z());
  EXPECT_EQ(IS1.Position().data()[0], p_vec[0]);
  EXPECT_EQ(IS1.Position().data()[1], p_vec[1]);
  EXPECT_EQ(IS1.Position().data()[2], p_vec[2]);
  EXPECT_EQ(IS1.Velocity().data()[0], v_vec[0]);
  EXPECT_EQ(IS1.Velocity().data()[1], v_vec[1]);
  EXPECT_EQ(IS1.Velocity().data()[2], v_vec[2]);
  EXPECT_EQ(IS1.GyroBias().data()[0], bg_vec[0]);
  EXPECT_EQ(IS1.GyroBias().data()[1], bg_vec[1]);
  EXPECT_EQ(IS1.GyroBias().data()[2], bg_vec[2]);
  EXPECT_EQ(IS1.AccelBias().data()[0], ba_vec[0]);
  EXPECT_EQ(IS1.AccelBias().data()[1], ba_vec[1]);
  EXPECT_EQ(IS1.AccelBias().data()[2], ba_vec[2]);

  // check quaternion/vector getters
  EXPECT_EQ(IS1.OrientationQuat().w(), q_quat.w());
  EXPECT_EQ(IS1.OrientationQuat().vec(), q_quat.vec());
  EXPECT_EQ(IS1.PositionVec(), p_vec);
  EXPECT_EQ(IS1.VelocityVec(), v_vec);
  EXPECT_EQ(IS1.GyroBiasVec(), bg_vec);
  EXPECT_EQ(IS1.AccelBiasVec(), ba_vec);

  // instantiate class with default state values
  bs_common::ImuState IS2(ros::Time(1));

  // check default state values
  EXPECT_EQ(IS2.Stamp(), ros::Time(1));
  EXPECT_EQ(IS2.Orientation().data()[0], 1);
  EXPECT_EQ(IS2.Orientation().data()[1], 0);
  EXPECT_EQ(IS2.Orientation().data()[2], 0);
  EXPECT_EQ(IS2.Orientation().data()[3], 0);
  EXPECT_EQ(IS2.Position().data()[0], 0);
  EXPECT_EQ(IS2.Position().data()[1], 0);
  EXPECT_EQ(IS2.Position().data()[2], 0);
  EXPECT_EQ(IS2.Velocity().data()[0], 0);
  EXPECT_EQ(IS2.Velocity().data()[1], 0);
  EXPECT_EQ(IS2.Velocity().data()[2], 0);
  EXPECT_EQ(IS2.GyroBias().data()[0], 0);
  EXPECT_EQ(IS2.GyroBias().data()[1], 0);
  EXPECT_EQ(IS2.GyroBias().data()[2], 0);
  EXPECT_EQ(IS2.AccelBias().data()[0], 0);
  EXPECT_EQ(IS2.AccelBias().data()[1], 0);
  EXPECT_EQ(IS2.AccelBias().data()[2], 0);

  // check quaternion/vector setters
  IS2.SetOrientation(q_quat);
  IS2.SetPosition(p_vec);
  IS2.SetVelocity(v_vec);
  IS2.SetGyroBias(bg_vec);
  IS2.SetAccelBias(ba_vec);

  EXPECT_EQ(IS2.Orientation().data()[0], q_quat.w());
  EXPECT_EQ(IS2.Orientation().data()[1], q_quat.x());
  EXPECT_EQ(IS2.Orientation().data()[2], q_quat.y());
  EXPECT_EQ(IS2.Orientation().data()[3], q_quat.z());
  EXPECT_EQ(IS2.Position().data()[0], p_vec[0]);
  EXPECT_EQ(IS2.Position().data()[1], p_vec[1]);
  EXPECT_EQ(IS2.Position().data()[2], p_vec[2]);
  EXPECT_EQ(IS2.Velocity().data()[0], v_vec[0]);
  EXPECT_EQ(IS2.Velocity().data()[1], v_vec[1]);
  EXPECT_EQ(IS2.Velocity().data()[2], v_vec[2]);
  EXPECT_EQ(IS2.GyroBias().data()[0], bg_vec[0]);
  EXPECT_EQ(IS2.GyroBias().data()[1], bg_vec[1]);
  EXPECT_EQ(IS2.GyroBias().data()[2], bg_vec[2]);
  EXPECT_EQ(IS2.AccelBias().data()[0], ba_vec[0]);
  EXPECT_EQ(IS2.AccelBias().data()[1], ba_vec[1]);
  EXPECT_EQ(IS2.AccelBias().data()[2], ba_vec[2]);

  // check array setters
  IS2.SetOrientation(IS1.Orientation().data());
  IS2.SetPosition(IS1.Position().data());
  IS2.SetVelocity(IS1.Velocity().data());
  IS2.SetGyroBias(IS1.GyroBias().data());
  IS2.SetAccelBias(IS1.AccelBias().data());

  EXPECT_EQ(IS2.Orientation().data()[0], q_quat.w());
  EXPECT_EQ(IS2.Orientation().data()[1], q_quat.x());
  EXPECT_EQ(IS2.Orientation().data()[2], q_quat.y());
  EXPECT_EQ(IS2.Orientation().data()[3], q_quat.z());
  EXPECT_EQ(IS2.Position().data()[0], p_vec[0]);
  EXPECT_EQ(IS2.Position().data()[1], p_vec[1]);
  EXPECT_EQ(IS2.Position().data()[2], p_vec[2]);
  EXPECT_EQ(IS2.Velocity().data()[0], v_vec[0]);
  EXPECT_EQ(IS2.Velocity().data()[1], v_vec[1]);
  EXPECT_EQ(IS2.Velocity().data()[2], v_vec[2]);
  EXPECT_EQ(IS2.GyroBias().data()[0], bg_vec[0]);
  EXPECT_EQ(IS2.GyroBias().data()[1], bg_vec[1]);
  EXPECT_EQ(IS2.GyroBias().data()[2], bg_vec[2]);
  EXPECT_EQ(IS2.AccelBias().data()[0], ba_vec[0]);
  EXPECT_EQ(IS2.AccelBias().data()[1], ba_vec[1]);
  EXPECT_EQ(IS2.AccelBias().data()[2], ba_vec[2]);

  // check scalar setters
  IS2.SetOrientation(q_quat.w(), q_quat.x(), q_quat.y(), q_quat.z());
  IS2.SetPosition(p_vec[0], p_vec[1], p_vec[2]);
  IS2.SetVelocity(v_vec[0], v_vec[1], v_vec[2]);
  IS2.SetGyroBias(bg_vec[0], bg_vec[1], bg_vec[2]);
  IS2.SetAccelBias(ba_vec[0], ba_vec[1], ba_vec[2]);

  EXPECT_EQ(IS2.Orientation().data()[0], q_quat.w());
  EXPECT_EQ(IS2.Orientation().data()[1], q_quat.x());
  EXPECT_EQ(IS2.Orientation().data()[2], q_quat.y());
  EXPECT_EQ(IS2.Orientation().data()[3], q_quat.z());
  EXPECT_EQ(IS2.Position().data()[0], p_vec[0]);
  EXPECT_EQ(IS2.Position().data()[1], p_vec[1]);
  EXPECT_EQ(IS2.Position().data()[2], p_vec[2]);
  EXPECT_EQ(IS2.Velocity().data()[0], v_vec[0]);
  EXPECT_EQ(IS2.Velocity().data()[1], v_vec[1]);
  EXPECT_EQ(IS2.Velocity().data()[2], v_vec[2]);
  EXPECT_EQ(IS2.GyroBias().data()[0], bg_vec[0]);
  EXPECT_EQ(IS2.GyroBias().data()[1], bg_vec[1]);
  EXPECT_EQ(IS2.GyroBias().data()[2], bg_vec[2]);
  EXPECT_EQ(IS2.AccelBias().data()[0], ba_vec[0]);
  EXPECT_EQ(IS2.AccelBias().data()[1], ba_vec[1]);
  EXPECT_EQ(IS2.AccelBias().data()[2], ba_vec[2]);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
