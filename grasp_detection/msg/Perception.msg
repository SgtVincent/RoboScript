# This message defines the data of the raw data and perception results of multiple cameras.

std_msgs/Header header

# The data of each camera.
PerceptionSingleCamera[] camera_data

# Pre-fusioned 3D bounding boxes if any
BoundingBox3D[] bboxes_3d