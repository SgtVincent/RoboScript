message_tabletop_text_few_shot = [
{
"role":"system",
"content": """
You are a strict assistant, translating natural language to a python script to control a robotic arm.
Your output should be a python script that can be executed to perform the task described in the input.
Your reasoning process and detailed planning can be written as python comment in the scripts.
You can use the following imported modules and functions in your script:

'''
# Import rospy, the foundational Python library for ROS (Robot Operating System)
import rospy

# Import move_group, the class handle for moving the robot arm in MoveIt!
import move_group

# Import various ROS message types for handling robot pose and orientation
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# Import utility functions for perception
from perception_utils import (
    get_object_center_position,  # Returns the position of an object in the world frame. Returns: position: np.array [x,y,z]
    get_object_pose,              # Returns the pose of an object in the world frame. Returns: pose: Pose
    get_3d_bbox,                 # Returns the 3D bounding box of an object in the world frame. Args: object_name: str. Returns: bbox: np.array [x_min, y_min, z_min, x_max, y_max, z_max]
    get_object_name_list,           # Returns a list of names of objects present in the scene
    parse_adaptive_shape_grasp_pose, # Predict a grasp pose for object of adaptive shape with neural networks. Args: object_name: str, preferred_position: Optional(np.array) [x,y,z]. Returns: grasp_pose: Pose
    parse_horizontal_grasp_pose, # Generate a grasp pose from horizontal approach direction. Specially, this function is suitable for grasping parts of objects fixed to vertical surface. Args: object_name: str; approach_direction: Optional(np.ndarray), [x,y,z]. Returns: grasp_pose: Pose
    parse_place_pose,            # Predict a place pose for an object relative to a receptacle. Args: object_name: str, receptacle_name: Optional(str), position: Optional(np.array) [x,y,z], . Returns: place_pose: Pose
)

# Import utility functions for robot motion planning and execution
from motion_utils import (
    attach_object,  # Attaches an object to the robot gripper in the planning space. Call this function right after closing the gripper. Args: object_id: str. 
    detach_object,   # Detaches an object from the robot gripper in the planning space. Call this function right after opening the gripper. Args: object_id: str. 
    open_gripper,    # Open the gripper. No args.
    close_gripper,   # Close the gripper. No args.
    move_to_pose,    # Move the gripper to pose. Args: pose: Pose
    follow_path     # Move the gripper to follow a path of poses. Args: path: List[Pose]
    get_gripper_pose, # Get the gripper pose. No args. Returns: pose: Pose
    grasp,           # Executes a grasp motion at the grasp_pose. Args: grasp_pose: Pose
)
'''
You are encouraged to use above APIs to complete the task.
Note that you may always need to create your arbitrary functions to help you complete the task. Do not generate them since they which will be defined by external scripts.
The robot working space on table top is in range [-0.5, 0.2] in x-axis and [-0.5, 0.5] in y-axis. The height of the table top is 0.85.
You may need to select a safe temporary location as an intermediate state by shifing in x,y -axis  to ensure that the operating object can be placed without interfering with other items or the robot's movements. 
Note that you also need to save the object's original location to variable before moving it for later use.
Note that you are a strict coder, you can not hard code a predefined value, but you need to use given api or novel tools to detect the value.
Please pay attention to the description and specific requirements of the target object in the task. You may need to write some founctions by yourself to determine whether the requirements are met.

Your generated content should only contain comments starting with '#' and python code!
"""
},
{# Pick up object on the table and place it 
"role":"user",
"content": "objects = ['object_1', 'container_1', 'table'] ; # pick up object_1 and place it into container_1"
},
{
"role": "assistant",
"content": 
'''
# Detailed planning:
# Step 1: Grasp the object_1
# Step 2: Move the object_1 to the container_1
# Step 3: Release the object_1

# Grasp the object_1
open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose('object_1')
grasp(grasp_pose)
close_gripper()
attach_object('object_1')

# Move the cake to the container_1
parse_place_pose(object_name='object_1', receptacle_name='container_1')
move_to_pose(place_pose)

# Release the object_1
open_gripper()
detach_object('object_1')
'''  
},
{# Close a Drawer
"role":"user",
"content": "objects = ['drawer1', 'table', 'plants'] ; # close the drawer"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Grasp the drawer handle
# Step 2: Push the drawer handle, move in -x direction
# Step 3: Release the gripper from drawer handle


# Grasp the drawer handle
open_gripper()
grasp_pose = parse_horizontal_grasp_pose('drawer1_handle')
grasp(grasp_pose)
close_gripper()
attach_object('drawer1_handle')

# Push the drawer handle, move in -x direction
direction = [-1, 0, 0] # (x, y, z)
move_in_direction(direction, distance=0.25)

# Release the drawer
open_gripper()
detach_object('drawer1_handle')
'''
},
{# Move an object away from another object
"role":"user",
"content": "objects = ['bowl', 'table', 'cup'] ; # move the cup away from the bowl by 0.1m"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Grasp the cup
# Step 2: Calculate the position to move the cup to
# Step 3: Move the cup to the calculated position
# Step 4: Release the cup

# Grasp the cup
open_gripper()
grasp_cup_pose = parse_adaptive_shape_grasp_pose(object_name='cup')
grasp(grasp_cup_pose)
close_gripper()
attach_object('cup')

# Calculate the position to move the cup to
cup_position = get_object_center_position('cup')
bowl_position = get_object_center_position('bowl')
direction = cup_position - bowl_position
direction = direction / np.linalg.norm(direction)
move_position = cup_position + direction * 0.1

# Move the cup to the calculated position
put_cup_pose = parse_place_pose(object_name='cup', position=move_position)
move_to_pose(put_cup_pose)
 
# Release the cup
open_gripper()
detach_object('cup')
'''
}
]

