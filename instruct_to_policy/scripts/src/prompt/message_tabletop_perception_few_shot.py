message_tabletop_perception = [
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
    get_obj_name_list,           # Returns a list of names of objects present in the scene
    parse_adaptive_shape_grasp_pose, # Parse adaptive grasp pose for objects. Args: object_name: str, preferred_position: Optional(np.ndarray); preferred gripper tip point position; preferred_approach_direction: Optional(np.ndarray), preferred gripper approach direction; preferred_plane_normal: Optional(np.ndarray), preferred gripper plane normal direction. Returns: grasp_pose: Pose
    parse_central_lift_grasp_pose, # This method involves a vertical lifting action. The gripper closes at the center of the object and is not suitable for elongated objects and is not suitable for the objects with openings, as the gripper's width is really small. It is optimal for handling spherical and cuboid objects without any opening that are not ideal for off-center grasping. Args: object_name: str, description: Optional(str) in ['top', 'center'], Returns: grasp_pose: Pose
    parse_place_pose,            # Predict the place pose for an object relative to a receptacle. Args: object_name: str, receptacle_name: Optional(str), position: Optional(np.array) [x,y,z], . Returns: place_pose: Pose
    detect_objects,              # Detect and update task-relevant objects' status in the environment. Call this function before interaction with environment objects. Args: object_list: Optional(List[str]), objects to detect.
    get_object_joint_info,       # Get the joint info of an object closest to a given position. Args: obj_name: str, name of the object; position: np.ndarray, select the joint closest to this position; type: str, allowed type of the joint, choice in ["any", "revolute", "prismatic"]. Returns: joint_info: dict, joint info of the object. {"joint_position":[x,y,z],"joint_axis":[rx,ry,rz],"type":str}
)

There are three preferences for function parse_adaptive_shape_grasp_pose(). Note that you need to choose the right preferences for different tasks and objects.
preferred_position: Optional(np.ndarray), preferred gripper tip point position. 
preferred_approach_direction: Optional(np.ndarray), preferred gripper approach direction.
preferred_plane_normal: Optional(np.ndarray), preferred gripper plane normal direction. This preference selects the grasp pose so that the gripper is parallel to the plane defined by the normal. It also means the gripper grasping at an axis with perpendicular pose to the axis. 

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
Note that you may always need to create your arbitrary functions to help you complete the task, which will be defined by external scripts.
The robot working space on table top is in range [-0.5, 0.2] in x-axis and [-0.5, 0.5] in y-axis. The height of the table top is 1.05.
You may need to select a safe temporary location by shifing in x,y -axis  to ensure that the operating object can be placed without interfering with other items or the robot's movements. 
Note that you are a strict coder, you can not hard code a predefined value, but you need to use api and tools to detect the value.
Please pay attention to the description and specific requirements of the target object in the task. You may need to write some founctions by yourself to determine whether the requirements are met.

Your generated content should only contain comments starting with '#' and python code!
"""
},
{# Pick up object on the table and place it 
"role":"user",
"content": "objects = ['object_1', 'container_1', 'object_random', 'container_random', 'table'] ; # pick up object_1 and place it into container_1"
},
{
"role": "assistant",
"content": 
'''
# Detailed planning:
# Step 0: Reason task-relevant objects
# Step 1: Grasp the object_1
# Step 2: Move the object_1 to the container_1
# Step 3: Release the object_1


# Reason task-relevant objects
objects = ['object_1', 'container_1', 'table']
detect_objects(object_list=objects)

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
"content": "objects = ['cabinet', 'drawer', 'handle', 'table', 'plants'] ; # close the first-top drawer"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 0: Reason task-relevant object, drawer and its handle, and detect objects 
# Step 1: Get the first-top drawer to be opened and all candidates of its handle position
# Step 3: Select the first handle as the target handle
# Step 4: Get the drawer prismatic joint closest to the handle position
# Step 5: Grasp the drawer handle with position and joint axis preferences
# Step 6: Push the drawer handle, move in -prismatic joint axis direction
# Step 7: Release the gripper from drawer handle


# Reason task-relevant objects, drawer and its handle, and detect objects
objects = ['drawer','handle']
detect_objects(object_list=objects)

# Get the first-top drawer to be opened and all candidates of its handle position
drawer_postion=get_object_center_position("drawer_0")
x_min,y_min,z_min,x_max,y_max,z_max=get_3d_bbox("drawer_0")
detected_objects=get_obj_name_list()
handles= [name for name in detected_objects if 'handle' in name]
drawer_handles=[]
for handle_name in handles:
    if get_object_center_position(handle_name)[2]>z_min and get_object_center_position(handle_name)[2]<z_max:
        drawer_handles.append(handle_name)
    
# Select the first handle as the target handle
drawer_handle_name=drawer_handles[0]
drawer_handle_position = get_object_center_position(drawer_handle_name)

# Get the drawer prismatic joint closest to the handle position
drawer_handle_position = get_object_center_position(drawer_handle_name)
drawer_joint_info = get_object_joint_info(obj_name=drawer_name, position=drawer_handle_position, type="prismatic")
drawer_joint_position = drawer_joint_info["joint_position"]
drawer_joint_axis = drawer_joint_info["joint_axis"]

# Grasp the drawer handle with position and joint axis preferences
open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose(object_name=drawer_handle_name, preferred_position=drawer_handle_position, preferred_approach_direction=drawer_joint_axis)
grasp(grasp_pose)
close_gripper()
attach_object(drawer_handle_name)

# Push the drawer handle, move in -prismatic joint axis direction
direction = -drawer_joint_axis
move_in_direction(direction, distance=0.2)

# Release the drawer
open_gripper()
detach_object(drawer_handle_name)
'''
},
{# Open a Door 
"role":"user",
"content": "objects = ['door', 'door_handle'] ; # open the door"
},
{
"role":"assistant",
"content":
'''
# Detailed planning: 
# Step 0: Reason task-relevant objects, and detect objects
# Step 1: Get the door to be opened and its closest handle position
# Step 2: Get the door revolute joint closest to the handle position
# Step 3: Grasp the door handle with position and joint axis preferences
# Step 4: Generate a rotational motion plan around the revolute joint 
# Step 5: Move the gripper along the motion plan
# Step 6: Release the gripper from door handle


# Reason task-relevant objects, and detect objects
objects = ['door', 'door_handle']
detect_objects(object_list=objects)

# Get the door to be opened and its closest handle position
door_position=get_object_center_position("door_0")
x_min,y_min,z_min,x_max,y_max,z_max=get_3d_bbox("door_0")
detected_objects=get_obj_name_list()
handles= [name for name in detected_objects if 'handle' in name]
hanle_distances=[]
for handle_name in handles:
    handle_position=get_object_center_position(handle_name)
    hanle_distances.append(np.linalg.norm(handle_position-door_position))
door_handle_name=handles[np.argmin(hanle_distances)]
handle_position=get_object_center_position(door_handle_name)
    
# Get the door revolute joint closest to the handle position
door_joint_info = get_object_joint_info(obj_name=door_name, position=handle_position, type="revolute")
door_joint_position = door_joint_info["joint_position"]
door_joint_axis = door_joint_info["joint_axis"]

# Grasp the door handle with position and joint axis preferences
open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose(object_name=handle_name, preferred_position=handle_position, preferred_approach_direction=door_joint_axis)
grasp(grasp_pose)
close_gripper()
attach_object(handle_name)

# Generate a rotational motion plan around the revolute joint
motion_plan = generate_arc_path_around_joint(current_pose=get_gripper_pose(),joint_axis=door_joint_axis, joint_position=door_joint_position, n=10, angle=30)

# Move the gripper along the motion plan
follow_path(motion_plan)
 
# Release the door
open_gripper()
detach_object(handle_name)
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
# Step 0: Reason task-relevant objects
# Step 1: Grasp the apple
# Step 2: Calculate the position to move the apple to
# Step 3: Move the apple to the calculated position
# Step 4: Release the apple


# Reason task-relevant objects
objects = ['bowl', 'table', 'apple']
detect_objects(object_list=objects)

# Grasp the apple
open_gripper()
grasp_apple_pose = parse_central_lift_grasp_pose(object='apple')
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
},
]

