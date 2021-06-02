# id of the lidar. This is the sensor id that the point are expressed in
uint8 sensor_id

# id of the current scan
uint64 measurement_id

# time stamp associated with the lidar data
time stamp

# Transform from baselink frame to world frame, measured from the sensor
#  with id: sensor_id
# Expressed as [R | t], read from left to right to down. Size is (3 x 4)
float32[] T_WORLD_BASELINK

# number of points in data
uint32 size             

# Type of lidar point, options: 0 - GENERAL, 1 - EDGE, 2 - SURFACE
uint8 type   

# x y z point coordinates expressed in the frame specified 
# in the header. Size is (size x 3)
float32[] points
