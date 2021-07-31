# Type of lidar point, options: 0 - GENERAL, 1 - EDGE strong, 2 - SURFACE strong, 
# 3 - edge weak, 4 - surface weak
uint8 point_type   

# frame id associated with scan data
string frame_id

# x y z point coordinates expressed in the frame specified above
# Size is: num_points x 3
float32[] points
