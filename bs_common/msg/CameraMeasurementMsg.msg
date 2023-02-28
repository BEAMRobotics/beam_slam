# header for camera measurement
std_msgs/Header header  

# image at slam chunk timestamp.
sensor_msgs/Image image

# all landmarks detected in this image
LandmarkMeasurementMsg[] landmarks 

# see DescriptorTypeStringMap in beam_cv/descriptors/Descriptor.h
string descriptor_type

# id of the camera (default 0)
uint8 sensor_id