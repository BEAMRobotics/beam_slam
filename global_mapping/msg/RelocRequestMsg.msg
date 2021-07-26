# recent scan size is (size x 3)
float32 [] scan

# recent image
sensor_msgs/Image image

# initial estimate of sensor pose provided to reloc.  Size is (3 x 4)
float32[] pose_estimate

# stamp associated with the scan/pose
time stamp