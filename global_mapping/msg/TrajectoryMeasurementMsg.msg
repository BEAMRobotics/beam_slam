# set of relative transforms between the keyframe associated with this slam chunk, 
# and some intermediate frame. This is often IMU measurements to get a higher 
$ frequency trajectory estimate. Each transform has [R | t] and is read from left 
# to right to down. Size is (num_poses x 3 x 4)
float32[] T_KEYFRAME_FRAME

# timestamps associated with each pose in nsecs
float64[] stamps