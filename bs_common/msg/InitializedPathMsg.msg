# Represent the results of an initialization process
Header header

# Represent the esiumated scale factor
float32 scale

# vector holding estimated gryoscope bias
geometry_msgs/Vector3 gyroscope_bias

# vector holding estimated accelerometer bias
geometry_msgs/Vector3 accelerometer_bias

# vector holding estimated gravity
geometry_msgs/Vector3 gravity

# vector of estimated poses form initialization
geometry_msgs/PoseStamped[] poses