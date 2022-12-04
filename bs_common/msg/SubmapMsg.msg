# lidar maps where points are expressed in the local mapper's world frame. 
LidarMeasurementMsg lidar_map

# 3D coordinates of visual map points
geometry_msgs/Vector3[] visual_map_points

# list of desriptors corresponding to the visual map points.
DescriptorMsg[] visual_map_descriptors

#list of word id's from the vocab corresponding to the descriptors
uint32[] visual_map_words

# see DescriptorTypeIntMap in beam_cv/descriptors/Descriptor.h
uint8 descriptor_type