message_parse_question = [
{
"role":"system",
"content": 
"""
You are a strict assistant, writing python scripts to calculate anwsers to questions.
Your output should be a python script that can be executed to perform the task described in the input.
You can use the following APIs:

'''
import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R

# Import various ROS message types for handling gripper pose and orientation
from geometry_msgs.msg import Pose, Point, Quaternion         

# Import utility functions for perception
from perception_utils import (
    get_object_center_position,  # Returns the position of an object in the world frame. Returns: position: np.array [x,y,z]
    get_object_pose,             # Returns the pose of an object in the world frame. Returns: pose: Pose
    get_3d_bbox,                 # Returns the 3D bounding box of an object in the world frame. Args: object_name: str. Returns: bbox: np.array [x_min, y_min, z_min, x_max, y_max, z_max]
    get_object_name_list,           # Returns a list of names of objects present in the scene
)
'''
The coordinate frame of the world/table is defined as follows:
front: +x, back: -x, left: -y, right: +y, up: +z, down: -z
You should use above APIs to calculate spatial relations between objects to answer the questions.
You should generate python code with the answer assigned to 'ret_val' variable.
"""
},
{# to the right 
"role": "user",
"content": "objects = ['yellow bowl', 'blue block', 'yellow block', 'blue bowl', 'fruit', 'green block', 'black bowl'] ; # is the blue block to the right of the yellow bowl"
},
{
"role": "assistant",
"content": '''
# Object on the right has a larger y coordinate 
ret_val = get_object_center_position('blue block')[1] > get_object_center_position('yellow bowl')[1]
'''
},
{# on
"role": "user",
"content": "objects = ['yellow bowl', 'blue block', 'yellow block', 'blue bowl', 'fruit', 'green block', 'black bowl'] ; # is the pink block on the green bowl"
},
{
"role": "assistant",
"content": '''
# Object above has a larger z coordinate
ret_val = get_object_center_position('pink block')[2] > get_object_center_position('green bowl')[2]
'''
},
{# inside
"role": "user",
"content": "objects = ['pink block', 'green block', 'pink bowl', 'blue block', 'blue bowl', 'green bowl'] ; # which objects are in the green bowl"
},
{
"role": "assistant",
"content": '''
# Use the bbox to check if the object is inside the green bowl
ret_val = []
objects = ['pink block', 'green block', 'pink bowl', 'blue block', 'blue bowl', 'green bowl']
# get the bbox of the green bowl
bbox_green_bowl = get_3d_bbox('green bowl')

for obj in objects:
    if obj != 'green bowl':
        # check if the object is inside the green bowl
        obj_center = get_object_center_position(obj)
        if bbox_green_bowl[0] < obj_center[0] < bbox_green_bowl[3] and bbox_green_bowl[1] < obj_center[1] < bbox_green_bowl[4] and bbox_green_bowl[2] < obj_center[2] < bbox_green_bowl[5]:
            ret_val.append(obj)
'''
},
{# same orientation
"role": "user",
"content": "objects = ['pink block', 'green block', 'pink bowl', 'blue block', 'blue bowl', 'green bowl'] ; Do blue bowl and green bowl have same orientation?"
},
{
"role": "assistant",
"content": '''
# Use the pose to check if the two bowls have same orientation
ret_val = []
objects = ['pink block', 'green block', 'pink bowl', 'blue block', 'blue bowl', 'green bowl']
# get the pose of the blue bowl
pose_blue_bowl = get_object_pose('blue bowl')
# get the pose of the green bowl
pose_green_bowl = get_object_pose('green bowl')

# get the rotation matrix of the blue bowl
r_blue_bowl = R.from_quat([pose_blue_bowl.orientation.x, pose_blue_bowl.orientation.y, pose_blue_bowl.orientation.z, pose_blue_bowl.orientation.w]).as_matrix()
# get the rotation matrix of the green bowl
r_green_bowl = R.from_quat([pose_green_bowl.orientation.x, pose_green_bowl.orientation.y, pose_green_bowl.orientation.z, pose_green_bowl.orientation.w]).as_matrix()
 
# check if the two bowls have same orientation
if np.allclose(r_blue_bowl, r_green_bowl):
    ret_val = True
else:
    ret_val = False
'''
}
]
