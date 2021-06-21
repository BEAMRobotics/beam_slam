# recent scan size is (size x 3)
float32 [] scan

### ^ should we have scans and image based requests in the same msg or separate?

# recent image
sensor_msgs/Image image

### ^ should this be a CameraMeasurementMsg?

# initial estimate of sensor pose provided to reloc.  Size is (3 x 4)
float32[] pose_estimate

# stamp associated with the scan/pose
time stamp