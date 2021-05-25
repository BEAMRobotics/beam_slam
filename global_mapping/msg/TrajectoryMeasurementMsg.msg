# id of this sensor whose pose is being measured
uint8 sensor_id                 

# id of the current measurement. Since trajectory measurements usually 
# poses between image or lidar keyframes, this should correspond to the 
# same keyframe measurement that it is attached to
uint64 measurement_id

# time stamp associated with the initial pose
time stamp

# Transform from sensor frame (associated with sensor_id) to world frame
# for  the sensor that these poses are measuring. 
# Expressed as [R | t], read from left to right to down. Size is (3 x 4)
float32[] T_WORLD_FRAME

# number of pose measurements  
uint32 size   

# set of transforms where each transform has [R | t] and is read 
# from left to right to down. Size is (size x 3 x 4)
float32[] poses

# timestamps associated with each pose
float64[] stamps