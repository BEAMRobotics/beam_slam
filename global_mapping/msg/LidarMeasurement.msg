# time stamp associated with the lidar data
time stamp

# sensor id that the point are expressed in
uint8 sensor_id

# number of points in data
uint32 size             

# Type of lidar point, options: GENERAL, EDGE, SURFACE
string type   

# x y z point coordinates expressed in the frame specified 
# in the header. Size is (size x 3)
float32[] data
