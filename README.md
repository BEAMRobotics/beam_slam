# beam_slam

## Description:

This repo contains beam's lidar-camera-inertial SLAM code. All other SLAM code is being replaced by this repo. beam_slam uses the fuse framework with our own custom implementations of cost functions and models to suit our framework.

## Dependencies:

* fuse: https://github.com/locusrobotics/fuse (kinetic-devel branch for ROS Kinetic, devel branch for ROS Melodic)
* tf2_2d: https://github.com/locusrobotics/tf2_2d.git
* qwt: sudo apt-get install libqwt-dev (for Kinetic or Melodic) or libqwt-qt5-dev (for Noetic)
* calibration_publisher: https://github.com/BEAMRobotics/beam_robotics/tree/master/calibration/calibration_publisher
* libbeam: https://github.com/BEAMRobotics/libbeam (globally installed)

If you want to run IMU tests, you will also need:

* basalt: https://github.com/BEAMRobotics/basalt-headers-mirror
* sophus: https://github.com/strasdat/Sophus (tested at commit 936265f)

## Compiling:

When compiling beam_slam modules on Ubuntu 18.04, specify the number of processors to be at most half of the number of processors (i.e. $(nproc)) available. For example:

`catkin build -j4` if `$(nproc)` is eight or higher

To compile everything needed to run SLAM follow these compilations:

```
catkin build -j3 calibration_publisher
catkin build -j3 fuse_optimizers
catkin build -j3 bs_models
```

## Running SLAM:

1. Create an extrinsics file for your robot, example [here](https://github.com/BEAMRobotics/beam_slam/blob/main/beam_slam_launch/calibrations/ig2/extrinsics.json).
2. Create a calibration launch file for these extrinsics, example [here](https://github.com/BEAMRobotics/beam_slam/blob/main/beam_slam_launch/launch/ig2/calibration_publisher_ig2.launch).
3. Create a calibration parameter file, example [here](https://github.com/BEAMRobotics/beam_slam/blob/add_documentation/beam_slam_launch/config/ig2/calibration_params.yaml).
4. Create a yaml config for your desired SLAM setup, example [here](https://github.com/BEAMRobotics/beam_slam/blob/main/beam_slam_launch/config/ig2/lvio.yaml). A detailed explanation of each sensor models parameters are found in the README under bs_models.
5. Create a launch file to run the fuse optimizer of your choice, example [here](https://github.com/BEAMRobotics/beam_slam/blob/main/beam_slam_launch/launch/ig2/lvio_local_mapper.launch), make sure to keep the name as local_mapper as message names depend on this naming convention
6. Lastly launch this launch file and run your rosbag with use_sim_time set to true.

---

## Known issues:

* Openmp:

When compiling fuse, if you get the following error:

```
/usr/bin/ld: /usr/local/lib/libceres.a(local_parameterization.cc.o): undefined reference to symbol 'omp_get_max_threads@@OMP_1.0'
//usr/lib/x86_64-linux-gnu/libgomp.so.1: error adding symbols: DSO missing from command line
collect2: error: ld returned 1 exit status
make[2]: *** [/userhome/catkin_ws/devel/.private/fuse_optimizers/lib/fuse_optimizers/fixed_lag_smoother_node] Error 1
```
Then add the following line to fuse_optimizer/CMakeLists.txt

```
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

```
