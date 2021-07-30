# Type of lidar point, options: 0 - GENERAL, 1 - EDGE, 2 - SURFACE
uint8 point_type   

# frame id associated with scan data
string frame_id

# x y z point coordinates expressed in the frame specified above
# Size is: num_points x 3
float32[] points
