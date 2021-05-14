# number of pose measurements  
uint32 size   

# set of transforms where each transform has [R | t] and is read 
# from left to right to down. Size is (size x 3 x 4)
float32[] data 

# target frame of pose transform. T_to_frame_from_frame
string to_frame 

# source frame of pose transform. T_to_frame_from_frame
string from_frame 