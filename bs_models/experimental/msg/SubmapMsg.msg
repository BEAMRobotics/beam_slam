# lidar maps where points are expressed in the local mapper's world frame. 
LidarMeasurementMsg lidar_map

# 3D coordinates of visual map points
geometry_msgs/Vector3[] visual_map_points

# list of word id's from the vocab corresponding to each visual map point
uint32[] visual_map_word_ids