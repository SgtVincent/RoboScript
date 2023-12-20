# This message defines the joint axes of a robot. 

# The name of the joint
std_msgs/String joint_name
# Axis direction
geometry_msgs/Vector3 axis
# The origin of axis vector 
geometry_msgs/Vector3 origin
# The joint type (same order as defined in Gazebo)
#   Joint_Type_REVOLUTE = 1,
#   Joint_Type_REVOLUTE2 = 2,
#   Joint_Type_PRISMATIC = 3,
#   Joint_Type_UNIVERSAL = 4,
#   Joint_Type_BALL = 5,
#   Joint_Type_SCREW = 6,
#   Joint_Type_GEARBOX = 7,
#   Joint_Type_FIXED = 8
###########################
uint8 type 


