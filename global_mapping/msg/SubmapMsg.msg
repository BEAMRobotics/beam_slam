# timestamp associated with submap response
time stamp

# submap selected from reloc process.  Size is (size x 3).
float32[] submap

# visual keypoints in local mapper's world frame.
# each keypoint is an instance of Descriptor.msg
Descriptor[] keypoints

### ^ separate descriptor message