#define CATCH_CONFIG_MAIN

#include <ctime>
#include <random>

#include <catch2/catch.hpp>
#include <fuse_core/constraint.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_graphs/hash_graph.h>

#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <basalt/imu/preintegration.h>
#include <basalt/spline/se3_spline.h>

using namespace beam_models::frame_to_frame;

TEST_CASE("Test preintegration with zero noise and zero bias") { 

  // set intrinsic noise of imu to zero
  Eigen::Matrix3d zero_intrinsic_noise{Eigen::Matrix3d::Zero()};
  ImuPreintegration::Params params;
  params.cov_w = zero_intrinsic_noise;
  params.cov_a = zero_intrinsic_noise;
  params.cov_bg = zero_intrinsic_noise;
  params.cov_ba = zero_intrinsic_noise;

  // instantiate preintegration class with zero noise. By default,
  // bias terms (i.e. bg,ba) are set to zero
  std::unique_ptr<ImuPreintegration> imu_preintegration = 
      std::make_unique<ImuPreintegration>(params);

  // generate random ground truth trajectory of imu using splines
  int num_knots = 15; // number of knots in spline curve
  int64_t start_time_ns = 0; // set start time to 0 seconds
  int64_t time_interval_ns = 10e9; // 10 seconds btw each knot
  
  // generate spline curve of fifth order.
  basalt::Se3Spline<5> gt_spline(time_interval_ns, start_time_ns);
  gt_spline.genRandomTrajectory(num_knots);

  // from ground truth spline curve, get pose and derivative relationships
  // necessary to create synthetic imu readings with zero noise and zero bias
  int64_t dt_ns = 1e7; // assume date at 100 Hz
  static const Eigen::Vector3d gravity(0, 0, -params.gravitational_acceleration);
  for (int64_t t_ns = start_time_ns + dt_ns / 2;
       t_ns < int64_t(20e9);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - gravity);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    // take measurement in the middle of the interval
    ros::Time t_ros((t_ns + 0.5*dt_ns)/1e9);
    std::cout << "rosTime: " << t_ros << " t_ns: " << t_ns << std::endl; 

    // populate imu data type with synthetic readings
    ImuPreintegration::ImuData imu_data;
    imu_data.time = t_ros;
    imu_data.angular_velocity = rot_vel_body;
    imu_data.linear_acceleration = accel_body;
    
    // populate buffer 
    imu_preintegration->PopulateBuffer(imu_data);
  }

  // get pose and velocity of imu wrt world at start 
  Sophus::SE3d start_pose = gt_spline.pose(start_time_ns); 
  Eigen::Vector3d start_velocity = gt_spline.transVelWorld(start_time_ns);

  // set start time of imu preintegration and instantiate first imu state 
  ros::Time start_t(0, start_time_ns);
  Eigen::Matrix3d start_orientation_mat = start_pose.so3().matrix();
  Eigen::Vector3d start_position = start_pose.translation();
  Eigen::Quaterniond start_orientation(start_orientation_mat);
  imu_preintegration->SetStart(start_t, start_orientation, start_velocity, start_position);

  // get imu state at start. This state remains unchanged during integration of imu readings.
  ImuState start_imu_state = imu_preintegration->GetImuState();

  // check first imu state against ground truth
  REQUIRE(start_imu_state.Stamp() == start_t);
  REQUIRE(start_imu_state.Orientation().data()[0] == start_orientation.w());
  REQUIRE(start_imu_state.Orientation().data()[1] == start_orientation.x());
  REQUIRE(start_imu_state.Orientation().data()[2] == start_orientation.y());
  REQUIRE(start_imu_state.Orientation().data()[3] == start_orientation.z());
  
  for (int i = 0; i < 3; i++) {
    REQUIRE(start_imu_state.Velocity().data()[i] == start_velocity[i]);
  }
  for (int i = 0; i < 3; i++) {
    REQUIRE(start_imu_state.Position().data()[i] == start_position[i]);
  }

  // integrate imu readings over the imu buffer. For testing purposes, lets calculate the 
  // new imu state after 10 seconds. For this test, it is not necessary to calculate the 
  // covariance matrix or jacobians. 
  REQUIRE(imu_preintegration->Integrate(10.0, false, false)); 

  // Now that the imu measurements have been integrated, we may estimate the new state
  // in the world coordinate frame
  ImuState end_imu_state = imu_preintegration->PredictState(start_imu_state);

  // get ground truth pose and velocity of imu wrt world at after 10 seconds 
  Sophus::SE3d end_pose = gt_spline.pose(time_interval_ns); 
  Eigen::Vector3d end_velocity = gt_spline.transVelWorld(time_interval_ns);

  Eigen::Matrix3d end_orientation_mat = start_pose.so3().matrix();
  Eigen::Vector3d end_position = start_pose.translation();
  Eigen::Quaterniond end_orientation(start_orientation_mat);

  // Check end conditions between imu preintegration and ground truth
  REQUIRE(end_imu_state.Orientation().data()[0] == Approx(end_orientation.w()).epsilon(1e-4) );
  REQUIRE(end_imu_state.Orientation().data()[1] == Approx(end_orientation.x()).epsilon(1e-4) );
  REQUIRE(end_imu_state.Orientation().data()[2] == Approx(end_orientation.y()).epsilon(1e-4) );
  REQUIRE(end_imu_state.Orientation().data()[3] == Approx(end_orientation.z()).epsilon(1e-4) );
  REQUIRE(end_imu_state.Velocity().data()[0] == Approx(end_velocity[0]).epsilon(1e-4) );
  REQUIRE(end_imu_state.Velocity().data()[1] == Approx(end_velocity[1]).epsilon(1e-4) );
  REQUIRE(end_imu_state.Velocity().data()[2] == Approx(end_velocity[2]).epsilon(1e-4) );
  REQUIRE(end_imu_state.Position().data()[0] == Approx(end_position[0]).epsilon(1e-4) );
  REQUIRE(end_imu_state.Position().data()[1] == Approx(end_position[1]).epsilon(1e-4) );
  REQUIRE(end_imu_state.Position().data()[2] == Approx(end_position[2]).epsilon(1e-4) );
}