# beam_slam

## Description:

This repo contains beam's lidar-camera-inertial SLAM code. All other SLAM code is being replaced by this repo. beam_slam uses the fuse framework with our own custom implementations of cost functions and models to suit our framework.

## Dependencies:

* fuse: https://github.com/locusrobotics/fuse (kinetic-devel branch for ROS Kinetic, devel branch for ROS Melodic)
* tf2_2d: https://github.com/locusrobotics/tf2_2d.git
* qwt: sudo apt-get install libqwt-dev
* basalt: https://github.com/BEAMRobotics/basalt-headers-mirror
* sophus: https://github.com/strasdat/Sophus (tested at commit 936265f)

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
* -j$(nproc):

When compiling beam_slam modules on Ubuntu 18.04, specify the number of processors to be at most half of the number of processors (i.e. $(nproc)) available. For example:

`catkin build -j4` if `$(nproc)` is eight or higher
