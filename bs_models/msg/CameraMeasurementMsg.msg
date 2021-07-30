# image at slam chunk timestamp.
sensor_msgs/Image image

# all landmarks detected in this image
LandmarkMeasurementMsg[] landmarks 

# see DescriptorTypeIntMap in beam_cv/descriptors/Descriptor.h
uint8 descriptor_type

# id of the camera (default 0)
uint8 sensor_id

# id of the current image
uint64 measurement_id