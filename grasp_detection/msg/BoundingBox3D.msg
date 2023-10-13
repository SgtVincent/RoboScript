# A 3D bounding box that can be positioned and rotated about its center (6 DOF)
# Dimensions of this box are in meters

# ID or label of the detected object
string object_id

# The 3D position and orientation of the bounding box center
geometry_msgs/Pose center

# The size of the bounding box, in meters, surrounding the object's center
geometry_msgs/Vector3 size