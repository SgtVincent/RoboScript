message_tabletop_perception_few_shot = [
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
    get_object_center_position,  # Returns the position of an object in the world frame. Args: object_name: str. Returns: position: np.array [x,y,z]
    get_object_pose,              # Returns the pose of an object in the world frame. Args: object_name: str. Returns: pose: Pose
    get_3d_bbox,                 # Returns the 3D bounding box of an object in the world frame. Args: object_name: str. Returns: bbox: np.array [x_min, y_min, z_min, x_max, y_max, z_max]
    get_object_name_list,           # Returns a list of names of objects present in the scene
    parse_adaptive_shape_grasp_pose, # Predict a grasp pose for object of adaptive shape with neural networks. Args: object_name: str, preferred_position: Optional(np.array) [x,y,z]. Returns: grasp_pose: Pose
    parse_horizontal_grasp_pose, # Generate a grasp pose from horizontal approach direction. Specially, this function is suitable for grasping parts of objects fixed to vertical surface. Args: object_name: str; approach_direction: Optional(np.ndarray), [x,y,z]. Returns: grasp_pose: Pose
    parse_place_pose,            # Predict the place pose for an object relative to a receptacle. Args: object_name: str; receptacle_name: Optional(str); position: Optional(np.array) [x,y,z], . Returns: place_pose: Pose
    detect_objects,              # Detect and update task-relevant objects' status in the environment. The function takes a list of object categories as input, and detects all instances of categories in the environment, creating instance name handles globally by attaching an instance id to the category name. For example, "apple" -> ["apple_0", "apple_1", ...]. Call this function before interaction with environment objects. Args: category_list: Optional(List[str]), object categories to detect.
)

# Import utility functions for robot motion planning and execution
from motion_utils import (
    attach_object,  # Attaches an object to the robot gripper in the planning space. Call this function right after closing the gripper. Args: object_id: str. 
    detach_object,   # Detaches an object from the robot gripper in the planning space. Call this function right after opening the gripper. Args: object_id: str. 
    open_gripper,    # Open the gripper. No args.
    close_gripper,   # Close the gripper. No args.
    move_to_pose,    # Move the gripper to pose. Args: pose: Pose
    follow_path,     # Move the gripper to follow a path of poses. Args: path: List[Pose]
    get_gripper_pose, # Get the gripper pose. No args. Returns: pose: Pose
    grasp,           # Executes a grasp motion at the grasp_pose. Args: grasp_pose: Pose
)
'''
You are encouraged to use above APIs to complete the task.
Note that you may always need to create your arbitrary functions to help you complete the task, which will be defined by external scripts.
The robot working space on table top is in range [-0.5, 0.2] in x-axis and [-0.5, 0.5] in y-axis. The height of the table top is 0.85.
You may need to select a safe temporary location by shifing in x,y -axis  to ensure that the operating object can be placed without interfering with other items or the robot's movements. 
Note that you are a strict coder, you can not hard code a predefined value, but you need to use api and tools to detect the value.
Please pay attention to the description and specific requirements of the target object in the task. You may need to write some founctions by yourself to determine whether the requirements are met.

Your generated content should only contain comments starting with '#' and python code!
"""
},
{# Pick up object on the table and place it 
"role":"user",
"content": "objects = ['object_A', 'container_B', 'object_random', 'container_random', 'table'] ; # pick up object_A and place it into container_B"
},
{
"role": "assistant",
"content": 
'''
# Detailed planning:
# Step 0: Reason task-relevant categories: ['object_A', 'container_B'], and detect object instances
# Step 1: Grasp the object_A
# Step 2: Move the object_A to the container_B
# Step 3: Release the object_A


# Reason task-relevant categories: ['object_A', 'container_B'], and detect object instances
object_categories = ['object_A', 'container_B']
detect_objects(object_categories)

# Grasp the object_A
open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose('object_A_0')
grasp(grasp_pose)
close_gripper()
attach_object('object_A_0')

# Move the object_A to the container_B
place_pooe = parse_place_pose('object_A_0', receptacle_name='container_B_0')
move_to_pose(place_pose)

# Release the object_A
open_gripper()
detach_object('object_A_0')
'''  
},
{# Close a Drawer
"role":"user",
"content": "objects = ['cabinet', 'cabinet.drawer', 'cabinet.handle', 'table', 'plants'] ; # close the drawer"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 0: Reason task-relevant categories: ['cabinet', 'cabinet.drawer', 'cabinet.handle'], and detect object instances
# Step 1: Get the drawer to be closed and its handle position
# Step 2: Get the drawer prismatic joint closest to the handle position
# Step 3: Grasp the drawer handle with negative joint axis as approach direction
# Step 4: Push the drawer handle, move in -prismatic joint axis direction
# Step 5: Release the gripper from drawer handle


# Reason task-relevant categories: ['cabinet', 'cabinet.drawer', 'cabinet.handle'], and detect object instances
object_categories = ['cabinet', 'cabinet.drawer', 'cabinet.handle']
detect_objects(object_categories)

# Get the drawer to be closed and its handle position
drawer_name = 'cabinet.drawer_0'
drawer_handle_name = 'cabinet.handle_0'

# Get the drawer prismatic joint closest to the handle position
drawer_handle_position = get_object_center_position(drawer_handle_name)
drawer_joint_info = get_object_joint_info(object_name=drawer_name, position=drawer_handle_position, type="prismatic")
drawer_joint_position = drawer_joint_info["joint_position"]
drawer_joint_axis = drawer_joint_info["joint_axis"]

# Grasp the drawer handle with negative joint axis as approach direction
open_gripper()
grasp_pose = parse_horizontal_grasp_pose(object_name=drawer_handle_name, approach_direction=-drawer_joint_axis)
grasp(grasp_pose)
close_gripper()
attach_object(drawer_handle_name)

# Push the drawer handle, move in -prismatic joint axis direction
direction = -drawer_joint_axis
move_in_direction(direction, distance=0.25)

# Release the drawer
open_gripper()
detach_object(drawer_handle_name)
'''
},
{# Move an object away from another object
"role":"user",
"content": "objects = ['bowl', 'table', 'apple', 'shelve', 'plants'] ; # move the apple away from the bowl by 0.1m"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 0: Reason task-relevant categories: ['bowl', 'table', 'apple'], and detect object instances
# Step 1: Grasp the apple
# Step 2: Calculate the position to move the apple to
# Step 3: Move the apple to the calculated position
# Step 4: Release the apple


# Reason task-relevant categories: ['bowl', 'table', 'apple'], and detect object instances
object_categories = ['bowl', 'table', 'apple']
detect_objects(object_categories)

# Grasp the apple
open_gripper()
grasp_apple_pose = parse_adaptive_shape_grasp_pose(object_name='apple_0')
grasp(grasp_apple_pose)
close_gripper()
attach_object('apple_0')

# Calculate the position to move the apple to
apple_position = get_object_center_position('apple_0')
bowl_position = get_object_center_position('bowl_0')
direction = apple_position - bowl_position
direction = direction / np.linalg.norm(direction)
move_position = apple_position + direction * 0.1

# Move the apple to the calculated position
put_apple_pose = parse_place_pose('apple_0', position=move_position)
move_to_pose(put_apple_pose)
 
# Release the apple
open_gripper()
detach_object('apple_0')
'''
},
]

