# set of relative transforms between the keyframe associated with this slam chunk, 
# and some intermediate frame (T_KEYFRAME_FRAME). This is often IMU measurements 
# to get a higher frequency trajectory estimate. Each transform has [R | t] and 
# is read from left to right to down. Size is (num_poses x 3 x 4). Note that 
# keyframes must all expressed in the baselink frame
float64[] poses

# timestamps associated with each pose in nsecs
float64[] stamps