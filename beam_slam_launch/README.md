# beam_slam_demo

## Description:

Package containing launch and config files for running beam_slam. This also contains example launch/config/test data for running demos. 

## Demo Instructions:

1.  Build package (i.e., `catkin build`)
2.  Run each of the following commands in a separate terminal:
    -  `roscore`
    -  `rosparam set /use_sim_time true`
    -  `rosbag play $(rospack find beam_launch)/test_bags/fuse_test_bag01.bag --pause --clock`
    -  `roslaunch beam_launch demo.launch`
