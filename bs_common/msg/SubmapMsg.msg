# lidar maps where points are expressed in the local mapper's world frame. 
geometry_msgs/Vector3[] lidar_map
geometry_msgs/Vector3[] lidar_edges_strong
geometry_msgs/Vector3[] lidar_edges_weak
geometry_msgs/Vector3[] lidar_surfaces_strong
geometry_msgs/Vector3[] lidar_surfaces_weak

# 3D coordinates of visual map points
geometry_msgs/Vector3[] visual_map_points

# list of desriptors corresponding to the visual map points.
DescriptorMsg[] visual_map_descriptors

# see DescriptorTypeIntMap in beam_cv/descriptors/Descriptor.h
uint8 descriptor_type