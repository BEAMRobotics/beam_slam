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
  int64_t time_simulation_ns = start_time_ns + 20e9; // duration over which data is captured

  // benchmark against basalt
  basalt::IntegratedImuMeasurement<double> imu_meas(start_time_ns, Eigen::Vector3d::Zero(),
                                                    Eigen::Vector3d::Zero());

  // generate spline curve of fifth order.
  basalt::Se3Spline<5> gt_spline(time_interval_ns, start_time_ns);
  gt_spline.genRandomTrajectory(num_knots);

  // from ground truth spline curve, get pose and derivative relationships
  // necessary to create synthetic imu readings with zero noise and zero bias
  int64_t dt_ns = 1e7; // assume data at 100 Hz
  static const Eigen::Vector3d gravity(0, 0, -params.gravitational_acceleration);
  for (int64_t t_ns = start_time_ns + dt_ns / 2;
       t_ns < time_simulation_ns;
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - gravity);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    // take measurement in the middle of the interval
    ros::Time t_ros((t_ns + 0.5*dt_ns)/1e9);

    // populate imu data type with synthetic readings
    ImuPreintegration::ImuData imu_data;
    imu_data.time = t_ros;
    imu_data.angular_velocity = rot_vel_body; // units rad/sec
    imu_data.linear_acceleration = accel_body; // units m/sec^2

    // populate buffer 
    imu_preintegration->PopulateBuffer(imu_data);

    // bench mark against basalt
    basalt::ImuData<double> data;
    data.accel = accel_body;
    data.gyro = rot_vel_body;
    data.t_ns = t_ns + dt_ns / 2;  // measurement in the middle of the interval;

    imu_meas.integrate(data, Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());
  }

  // get pose and velocity of imu wrt world at start 
  Sophus::SE3d start_pose = gt_spline.pose(start_time_ns); 
  Eigen::Vector3d start_velocity = gt_spline.transVelWorld(start_time_ns);

  // set start time of imu preintegration and instantiate first imu state 
  ros::Time start_t(start_time_ns/1e9);
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

  std::cout << "Start State: " << std::endl;
  start_imu_state.Print();

  // integrate imu readings over the imu buffer. For testing purposes, lets calculate the 
  // new imu state after 20 seconds. For this test, it is not necessary to calculate the 
  // covariance matrix or jacobians. 
  REQUIRE(imu_preintegration->Integrate(20.0, false, false)); 

  ImuPreintegration::Delta delta_state = imu_preintegration->GetDeltaState();
  std::cout << "Delta State: " << std::endl;
  std::cout << "          t: " << delta_state.t << std::endl;
  std::cout << "        q.w: " << delta_state.q.w() << std::endl;
  std::cout << "        q.x: " << delta_state.q.x() << std::endl;
  std::cout << "        q.y: " << delta_state.q.y() << std::endl;
  std::cout << "        q.z: " << delta_state.q.z() << std::endl;
  std::cout << "        v.x: " << delta_state.v[0]  << std::endl;
  std::cout << "        v.y: " << delta_state.v[1]  << std::endl;
  std::cout << "        v.z: " << delta_state.v[2]  << std::endl;
  std::cout << "        p.x: " << delta_state.p[0]  << std::endl;
  std::cout << "        p.y: " << delta_state.p[1]  << std::endl;
  std::cout << "        p.z: " << delta_state.p[2]  << std::endl;

  // benchmark against basalt
  basalt::PoseVelState<double> basalt_delta = imu_meas.getDeltaState();
  Eigen::Matrix3d basalt_end_orientation_mat  = basalt_delta.T_w_i.so3().matrix();
  Eigen::Vector3d basalt_end_velocity = basalt_delta.vel_w_i;
  Eigen::Vector3d basalt_end_position = basalt_delta.T_w_i.translation();
  Eigen::Quaterniond basalt_end_orientation(basalt_end_orientation_mat);
  std::cout << "Basalt Delta State: " << std::endl;
  std::cout << "                 t: " << imu_meas.get_dt_ns()/1e9 << std::endl;
  std::cout << "               q.w: " << basalt_end_orientation.w() << std::endl;
  std::cout << "               q.x: " << basalt_end_orientation.x() << std::endl;
  std::cout << "               q.y: " << basalt_end_orientation.y() << std::endl;
  std::cout << "               q.z: " << basalt_end_orientation.z() << std::endl;
  std::cout << "               v.x: " << basalt_end_velocity[0] << std::endl;
  std::cout << "               v.y: " << basalt_end_velocity[1] << std::endl;
  std::cout << "               v.z: " << basalt_end_velocity[2] << std::endl;
  std::cout << "               p.x: " << basalt_end_position[0] << std::endl;
  std::cout << "               p.y: " << basalt_end_position[1] << std::endl;
  std::cout << "               p.z: " << basalt_end_position[2] << std::endl;

  // Now that the imu measurements have been integrated, we may estimate the new state
  // in the world coordinate frame
  ImuState end_imu_state = imu_preintegration->PredictState(start_imu_state);

  // get ground truth pose and velocity of imu wrt world at after simulation 
  Sophus::SE3d end_pose = gt_spline.pose(time_simulation_ns-dt_ns); 
  Eigen::Vector3d end_velocity = gt_spline.transVelWorld(time_simulation_ns-dt_ns);

  Eigen::Matrix3d end_orientation_mat = end_pose.so3().matrix();
  Eigen::Vector3d end_position = end_pose.translation();
  Eigen::Quaterniond end_orientation(end_orientation_mat);

  std::cout << "End Ground Truth: " << std::endl;
  std::cout << "          t: " << (time_simulation_ns-dt_ns)/1e9 << std::endl;
  std::cout << "        q.w: " << end_orientation.w() << std::endl;
  std::cout << "        q.x: " << end_orientation.x() << std::endl;
  std::cout << "        q.y: " << end_orientation.y() << std::endl;
  std::cout << "        q.z: " << end_orientation.z() << std::endl;
  std::cout << "        v.x: " << end_velocity[0] << std::endl;
  std::cout << "        v.y: " << end_velocity[1] << std::endl;
  std::cout << "        v.z: " << end_velocity[2] << std::endl;
  std::cout << "        p.x: " << end_position[0] << std::endl;
  std::cout << "        p.y: " << end_position[1] << std::endl;
  std::cout << "        p.z: " << end_position[2] << std::endl;

  std::cout << "End State: " << std::endl;
  end_imu_state.Print();

  // Check end conditions between imu preintegration and ground truth
  REQUIRE(end_imu_state.Orientation().data()[0] == Approx(end_orientation.w()).epsilon(1e-1) );
  REQUIRE(end_imu_state.Orientation().data()[1] == Approx(end_orientation.x()).epsilon(1e-1) );
  REQUIRE(end_imu_state.Orientation().data()[2] == Approx(end_orientation.y()).epsilon(1e-1) );
  REQUIRE(end_imu_state.Orientation().data()[3] == Approx(end_orientation.z()).epsilon(1e-1) );
  REQUIRE(end_imu_state.Velocity().data()[0] == Approx(end_velocity[0]).epsilon(1e-1) );
  REQUIRE(end_imu_state.Velocity().data()[1] == Approx(end_velocity[1]).epsilon(1e-1) );
  REQUIRE(end_imu_state.Velocity().data()[2] == Approx(end_velocity[2]).epsilon(1e-1) );
  REQUIRE(end_imu_state.Position().data()[0] == Approx(end_position[0]).epsilon(1e-1) );
  REQUIRE(end_imu_state.Position().data()[1] == Approx(end_position[1]).epsilon(1e-1) );
  REQUIRE(end_imu_state.Position().data()[2] == Approx(end_position[2]).epsilon(1e-1) );
}
