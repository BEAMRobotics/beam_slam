# lidar measurement at current timestamp, expressed in the baselink frame
LidarMeasurementMsg lidar_measurement

# camera measurement
CameraMeasurementMsg camera_measurement

# initial estimate of sensor pose provided to reloc.  Size is: 3 x 4
float64[] T_WORLD_BASELINK

# stamp associated with the scan/pose
time stamp
