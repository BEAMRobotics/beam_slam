# lidar measurement at current timestamp, expressed in the baselink frame
LidarMeasurementMsg lidar_measurement

# camera measurement
CameraMeasurementMsg camera_measurement

# initial estimate of sensor pose provided to reloc.  Size is: 3 x 4
geometry_msgs/PoseStamped T_WORLD_BASELINK