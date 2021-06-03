# id associated with this landmark_id
uint64 landmark_id

# pixel coordinates where landmark was detected
float32 pixel_u
float32 pixel_v

# descriptor information. For descriptor type, see 
# libbeam/beam_cv/descriptors/Descriptor.h
uint8 descriptor_type
float32[] descriptor
