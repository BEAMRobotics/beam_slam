# lidar map where points are expressed in the local mapper's world frame. 
# Size is: num_lid_points x 3
float32[] lidar_map

# 3D coordinates of visual map points. Size is: num_vis_points x 3
float32[] visual_map_points

# list of desriptors corresponding to the visual map points. 
# Size is: num_vis_points x descriptor_size
float32[] visual_map_descriptors

# see DescriptorTypeIntMap in beam_cv/descriptors/Descriptor.h
uint8 descriptor_type