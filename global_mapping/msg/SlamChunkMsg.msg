LidarMeasurementMsg lidar_measurement
CameraMeasurementMsg camera_measurement
TrajectoryMeasurementMsg trajectory_measurement

# A timestamp associated with the slam chunk. Camera measurement should be from an 
# image with this timestamp, lidar measurements should be data motion compensated 
# to this timestamp, and trajectory measurement should have relative poses between 
# non-keyframes and the keyframe at this timestamp
time stamp         

# optimized baselink pose for this time instance. Transform has [R | t] and is read from left 
# to right to down. Size is (3 x 4)
float32[] T_WORLD_BASELINK

# stamp associated with the scan/pose
time stamp

# frame id associated with all pose and scan data
string baselink_frame_id