# The two ReFeree descriptors for one radar scan.

std_msgs/Header header

# Rotation-invariant: free space per range band. Used to find a place.
float32[] range_descriptor

# Per angular sector, and therefore not rotation invariant. Used to recover the
# yaw between two places once they have been matched.
float32[] angle_descriptor
