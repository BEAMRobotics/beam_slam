# lidar/loam data, expressed in the lidar frame 
LidarMeasurementMsg lidar_measurement

# camera measurement
CameraMeasurementMsg camera_measurement

# trajectory measurement: some intermediary frames between keyframes
nav_msgs/Path trajectory_measurement

# A timestamp associated with the slam chunk and the optimized baselink pose for this time instance.
# Camera measurement should be from an image with this timestamp, lidar data should be motion compensated to this 
# timestamp, and trajectory measurement should have relative poses between 
# non-keyframes and the keyframe at this timestamp
geometry_msgs/PoseStamped T_WORLD_BASELINK
