# id of this camera sensor
uint8 sensor_id                 

# id of the current image
uint64 image_id

# all landmarks detected in this image
LandmarkMeasurement[] landmarks 

# stamp associated with this image pose
time stamp

# Transform from sensor frame (associated with sensor_id) to world frame. 
# Expressed as [R | t], read from left to right to down. Size is (3 x 4)
float32[] T_WORLD_IMAGE            