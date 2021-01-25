# beam_launch

## Description:

Package containing all launch/config/test data for running beam slam. 


## Demo Instructions:

1. Build package (i.e., `catkin build`)
2. Run each of the following commands in a separate terminal:
   a. `roscore`
   b. `rosparam set /use_sim_time true`
   c. `rosbag play $(rospack find beam_launch)/test_bags/fuse_test_bag01.bag --pause --clock`
   d. `roslaunch beam_launch demo.launch`
