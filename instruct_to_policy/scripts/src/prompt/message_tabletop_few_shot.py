message_tabletop_ui = [
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
    get_object_center_position,  # Returns the current center position of an object in the world frame. The position of an object will change once you move it. Args: object_name: str. Returns: position: np.array [x,y,z]
    get_object_pose              # Returns the current pose of an object in the world frame. Args: object_name: str. Returns: pose: Pose
    get_3d_bbox,                 # Returns the current 3D bounding box of an object in the world frame. Args: object_name: str. Returns: bbox: np.array [x_min, y_min, z_min, x_max, y_max, z_max]
    get_obj_name_list,           # Returns a list of names of objects present in the scene. No args. Returns: object_name_list: List[str]
    parse_grasp_pose,            # Predict a grasp pose for a specified object with graspnet. Args: object_name: str, preferred_position: Optional(np.array) [x,y,z], preferred_direction: Optional(np.array) [vx, vy, vz]. Returns: grasp_pose: Pose
    parse_canonical_grasp_pose   # Predict a canonical (top-down) grasp pose for a specified object. Args: object_name: str, description: Optional(str) in ['top', 'center'], Returns: grasp_pose: Pose
    parse_horizontal_handle_grasp_pose # Predict a grasp pose for a horizontal handle. Args: object_name: str, Returns: grasp_pose: Pose
    parse_place_pose,            # Predict the place pose for an object relative to a receptacle. Args: object_name: str, receptacle_name: Optional(str), position: Optional(np.array) [x,y,z], . Returns: place_pose: Pose
)

# Import utility functions for robot motion planning and execution
from motion_utils import (
    attach_object,  # Attaches an object to the robot gripper in the planning space. Call this function right after closing the gripper. Args: object_id: str. 
    detach_object   # Detaches an object from the robot gripper in the planning space. Call this function right after opening the gripper. Args: object_id: str. 
    open_gripper    # Open the gripper. No args.
    close_gripper   # Close the gripper. No args.
    move_to_pose    # Move the gripper to pose. Args: pose: Pose
    get_gripper_pose # Get the gripper pose. No args. Returns: pose: Pose
    grasp           # Executes a grasp motion at the grasp_pose. Args: grasp_pose: Pose
)
'''
You are encouraged to use above APIs to complete the task.
Note that you may always need to create your arbitrary functions to help you complete the task. Do not generate them since they which will be defined by external scripts.
The robot working space on table top is in range [-0.5, 0.2] in x-axis and [-0.5, 0.5] in y-axis. The height of the table top is 1.05.
You may need to select a safe temporary location as an intermediate state by shifing in x,y -axis  to ensure that the operating object can be placed without interfering with other items or the robot's movements. 
Note that you also need to save the object's original location to variable before moving it for later use.
Note that you are a strict coder, you can not hard code a predefined value, but you need to use given api or novel tools to detect the value.
Please pay attention to the description and specific requirements of the target object in the task. You may need to write some founctions by yourself to determine whether the requirements are met.

Your generated content should only contain comments starting with '#' and python code!
"""
},
{# Pick up object on the table and place it 
"role":"user",
"content": "objects = ['object_1', 'container_1', 'table'] \n # pick up object_1 and place it into container_1"
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
grasp_pose = parse_grasp_pose('object_1')
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
"content": "objects = ['drawer1', 'table', 'plants'] \n # close the drawer"
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
grasp_pose = parse_horizontal_handle_grasp_pose('drawer1_handle')
grasp(grasp_pose)
close_gripper()
attach_object('drawer1_handle')

# Push the drawer handle, move in -x direction
direction = [-1, 0, 0] # (x, y, z)
move_in_direction(direction, distance=0.2)

# Release the drawer
open_gripper()
detach_object('drawer1_handle')
'''
},
{# Open a Door 
"role":"user",
"content": "objects = ['door'] \n # open the door"
},
{
"role":"assistant",
"content":
'''
# Detailed planning: 
# Step 1: Grasp the door handle 
# Step 2: Pull the door handle downwards with an arc path of small radius by 30 degrees
# Step 3: pull the door horizontally with an arc path of large radius by 60 degrees
# Step 4: release the door handle 

# Grasp the door handle
grasp_pose = parse_grasp_pose('door_handle')
grasp(grasp_pose)
close_gripper()
attach_object('door_handle')

# Pull the door handle downwards with an arc path of small radius by 30 degrees
current_pose = get_gripper_pose()
waypoints = generate_arc_path(current_pose, center=get_object_center_position('door_handle'), radius=0.1, n=20, angle=30)
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
move_group.execute(plan, wait=True)

# pull the door horizontally with an arc path of large radius by 60 degrees
current_pose = get_gripper_pose()
waypoints = generate_arc_path(current_pose, center=get_object_center_position('door_handle'), radius=0.5, n=40, angle=60)
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
move_group.execute(plan, wait=True)

# Release the door handle
open_gripper()
detach_object('door_handle')
'''
},
{# Move an object away from another object
"role":"user",
"content": "objects = ['bowl', 'table', 'apple'] \n # move the apple away from the bowl by 0.1m"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Grasp the apple
# Step 2: Calculate the position to move the apple to
# Step 3: Move the apple to the calculated position
# Step 4: Release the apple

# Grasp the apple
open_gripper()
grasp_apple_pose = parse_grasp_pose(object='apple')
grasp(grasp_apple_pose)
close_gripper()
attach_object('apple')

# Calculate the position to move the apple to
apple_position = get_object_center_position('apple')
bowl_position = get_object_center_position('bowl')
direction = apple_position - bowl_position
direction = direction / np.linalg.norm(direction)
move_position = apple_position + direction * 0.1

# Move the apple to the calculated position
put_apple_pose = parse_place_pose(object_name='apple', position=move_position)
move_to_pose(put_apple_pose)
 
# Release the apple
open_gripper()
detach_object('apple')
'''
}
]

