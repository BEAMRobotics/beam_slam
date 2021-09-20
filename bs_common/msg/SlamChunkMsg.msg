LidarMeasurementMsg lidar_measurement
CameraMeasurementMsg camera_measurement
TrajectoryMeasurementMsg trajectory_measurement

# A timestamp associated with the slam chunk. Camera measurement should be from an 
# image with this timestamp, lidar data should be motion compensated to this 
# timestamp, and trajectory measurement should have relative poses between 
# non-keyframes and the keyframe at this timestamp
time stamp         

# optimized baselink pose for this time instance. Transform has [R | t] and is read from left 
# to right to down. Size is (3 x 4)
float64[] T_WORLD_BASELINK
