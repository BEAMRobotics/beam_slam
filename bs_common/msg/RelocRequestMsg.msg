# lidar scan at current timestamp. Size is: num_points x 3
float32[] scan

# image at current timestamp.
sensor_msgs/Image image

# initial estimate of sensor pose provided to reloc.  Size is: 3 x 4
float32[] T_WORLD_BASELINK

# stamp associated with the scan/pose
time stamp
