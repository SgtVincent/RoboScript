message_fgen_few_shot = [
{
"role":"system",
"content": """
You are a strict assistant, translating natural language to a python script to define python functions.
The chat history provides some examples of how to define functions.

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
The robot working space on table top is in range [-0.5, 0.2] in x-axis and [-0.5, 0.5] in y-axis. The height of the table top is 0.85.
Your generated content should only contain a single Python function and comments starting with '#'. No questions or common texts.
"""
},
{# pick_and_place(pick_pose, place_pose)
"role":"user",
"content": "# define function: pick_and_place(object_name, pick_pose, place_pose)"
},
{
"role":"assistant",
"content": 
'''
def pick_and_place(object_name, pick_pose, place_pose):
    move_to_pose(pick_pose)
    close_gripper()
    attach_object(object_name)
    move_to_pose(place_pose)
    open_gripper()
    detach_object(object_name)
'''
},
{# move_in_direction(axis, distance)
"role":"user",
"content": "# define function: move_in_direction(axis=move_direction, distance=0.25)"
},
{
"role": "assistant",
"content":
'''
def move_in_direction(axis: np.array, distance: float):
    current_pose = get_gripper_pose()
    target_pose = Pose()
    normalized_axis = np.array(axis) / np.linalg.norm(axis)
    target_pose.position.x = axis[0] * distance + current_pose.position.x
    target_pose.position.y = axis[1] * distance + current_pose.position.y
    target_pose.position.z = axis[2] * distance + current_pose.position.z
    target_pose.orientation = current_pose.orientation
    move_to_pose(target_pose)
'''
},
{# check_object_on_receptacle(object_name, receptacle_name)
"role": "user",
"content": "# define function: check_object_on_receptacle(object_name='apple', receptacle_name='red_bowl')"
},
{
"role": "assistant",
"content": '''
def check_object_on_receptacle(object_name, receptacle_name):
    # Object above has a larger z coordinate
    return get_object_center_position(object_name)[2] > get_object_center_position(receptacle_name)[2]
'''
},
]

