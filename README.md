![beam_logo](https://github.com/BEAMRobotics/beam_slam/assets/25440002/c7dadb61-ef5a-4e50-b497-d7960349a468)

# beam_slam:

![ROS](https://img.shields.io/badge/ROS-noetic-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-purple)

beam_slam is a SLAM package developed by the [SRI lab](https://sri-lab.seas.ucla.edu/) at UCLA. While there are many SLAM algorithms available for various applications, beam_slam is particularly designed with infrastructure inspection in mind. To this end, it aims to decouple LIO and VIO as separate, high rate processes which both feed into a single global mapper which intelligently fuses the measurements from both subsystems, aiming to build the most accurate point cloud map possible for later inspections.

## Dependencies:

Most dependencies are included as submodules under the "dependencies" folder, however some additional dependencies are listed:
* qwt: sudo apt-get install libqwt-dev (for Kinetic or Melodic) or libqwt-qt5-dev (for Noetic)
* [calibration_publisher](https://github.com/BEAMRobotics/beam_robotics/tree/master/calibration/calibration_publisher) - include within the catkin workspace
* [libbeam](https://github.com/BEAMRobotics/libbeam) - installed globally, or within the catkin workspace

If you want to run IMU tests, you will also need:

* [sophus](https://github.com/strasdat/Sophus) - (tested at commit 936265f)

## Compiling:

To compile everything required to run beam_slam launch files use the following command:

`catkin build -j4 -DCMAKE_BUILD_TYPE=Release libbeam bs_optimizers bs_publishers bs_models calibration_publisher graph_rviz_plugin`

## Running SLAM:

1. Create an extrinsics file for your robot, example [here](https://github.com/BEAMRobotics/beam_slam/blob/main/beam_slam_launch/calibrations/ig2/extrinsics.json).
2. Create a calibration launch file for these extrinsics, example [here](https://github.com/BEAMRobotics/beam_slam/blob/main/beam_slam_launch/launch/ig2/calibration_publisher_ig2.launch).
3. Create a calibration parameter file, example [here](https://github.com/BEAMRobotics/beam_slam/blob/add_documentation/beam_slam_launch/config/ig2/calibration_params.yaml).
4. Create a yaml config for your desired SLAM setup, example [here](https://github.com/BEAMRobotics/beam_slam/blob/main/beam_slam_launch/config/ig2/lvio.yaml). A detailed explanation of each sensor models parameters are found in the README under bs_models.
5. Create a launch file to run the fuse optimizer of your choice, example [here](https://github.com/BEAMRobotics/beam_slam/blob/main/beam_slam_launch/launch/ig2/lvio.launch), make sure to keep the name as local_mapper as message names depend on this naming convention
---

Additionally, rviz configurations are included [here](https://github.com/BEAMRobotics/beam_slam/tree/main/beam_slam_launch/rviz) for each specific modality (vio, lio, lvio)

## Known issues:

* Openmp:

When compiling fuse, if you get the following error:

```
/usr/bin/ld: /usr/local/lib/libceres.a(local_parameterization.cc.o): undefined reference to symbol 'omp_get_max_threads@@OMP_1.0'
//usr/lib/x86_64-linux-gnu/libgomp.so.1: error adding symbols: DSO missing from command line
collect2: error: ld returned 1 exit status
make[2]: *** [/userhome/catkin_ws/devel/.private/fuse_optimizers/lib/fuse_optimizers/fixed_lag_smoother_node] Error 1
```
Then apply the patch FuseOpenMP.patch by:

`cd dependencies/fuse; git apply ../../FuseOpenMP.patch;`
