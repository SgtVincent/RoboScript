message_fgen= [
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
    get_object_pose              # Returns the pose of an object in the world frame. Returns: pose: Pose
    get_3d_bbox,                 # Returns the 3D bounding box of an object in the world frame. Args: object_name: str. Returns: bbox: np.array [x_min, y_min, z_min, x_max, y_max, z_max]
    get_obj_name_list,           # Returns a list of names of objects present in the scene
    parse_adaptive_shape_grasp_pose, # Args: object_name: str, preferred_position: Optional(np.array) [x,y,z], preferred_direction: Optional(np.array) [vx, vy, vz]. Returns: grasp_pose: Pose
    parse_central_lift_grasp_pose # Args: object_name: str, description: Optional(str) in ['top', 'center'], Returns: grasp_pose: Pose
    parse_horizontal_grasp_pose # Args: object_name: str, Returns: grasp_pose: Pose
    parse_place_pose,            # Predict the place pose for an object relative to a receptacle. Args: object_name: str, receptacle_name: Optional(str), position: Optional(np.array) [x,y,z], . Returns: place_pose: Pose
)

There are three functions for predicting grasp poses, each tailored for different kinds of objects. Note that you need to choose the right grasp function for the object carefully!!!
1. 'parse_central_lift_grasp_pose': This method involves a vertical lifting action. The gripper closes at the center of the object and is not suitable for elongated objects and is not suitable for the objects with openings, as the gripper's width is really small. It is optimal for handling spherical and cuboid objects without any opening that are not ideal for off-center grasping.
2. 'parse_horizontal_grasp_pose': This approach is designed for lateral engagement and is ideal for interacting with objects attached to surfaces perpendicular to the tabletop, commonly found in domestic or industrial environments.
3. 'parse_adaptive_shape_grasp_pose': This function utilizes GraspNet for predictive positioning. This technique is particularly effective for securing objects with unconventional shapes or openings, typical in various everyday scenarios.

# Import utility functions for robot motion planning and execution
from motion_utils import (
    attach_object,  # Attaches an object to the robot gripper in the planning space. Call this function right after closing the gripper
    detach_object   # Detaches an object from the robot gripper in the planning space. Call this function right after opening the gripper.
    open_gripper    # Open the gripper 
    close_gripper   # Close the gripper
    move_to_pose    # Move the gripper to pose.
    get_gripper_pose # Get the gripper pose
    grasp           # Executes a grasp motion at the grasp_pose. Args: grasp_pose: Pose
)
'''
The robot working space on table top is in range [-0.5, 0.2] in x-axis and [-0.5, 0.5] in y-axis. The height of the table top is 1.05.
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
"content": "# define function: move_in_direction(axis=move_direction, distance=0.2)"
},
{
"role": "assistant",
"content":
'''
def move_in_direction(self, axis: np.array, distance: float, reuse_dict: dict = None):
    """
    reuse_dict = {'object_name', 'choice'}
        - object_name: the object being operated
        - choice includes 'save', 'forward', 'reversed', 'averaged'
            - save: save the initial pose and final pose of the {object_name} for reuse, such as save the position when opening the drawer
            - forward: reuse the saved position of {object_name} in forward direction, setting the initial and final poses as the saved initial and final poses, respectively
            - reversed: reuse the saved position of {object_name} in reversed direction, setting the initial and final poses as the saved final and initial poses, respectively
            - averaged: reuse the averaged saved position of {object_name}, setting the final poses as the averaged pose of the saved final and initial poses
    """
    print("___________move_in_direction_____________")
    waypoints = []
    current_pose = self.move_group.get_current_pose().pose
    if reuse_dict == None:
        is_calculate = True
    elif reuse_dict['choice'] == 'save':
        is_calculate = True
    else:
        is_calculate = False

    if is_calculate == True:
        target_pose = copy.deepcopy(current_pose)
        target_pose.position.x += axis[0] * distance
        target_pose.position.y += axis[1] * distance
        target_pose.position.z += axis[2] * distance
    else:
        object_name = reuse_dict['object_name']
        choice = reuse_dict['choice']
        if choice == 'save':
            object_pos_dict = {'initial_pose': current_pose, 'final_pose': target_pose}
            self.pos_dict_for_reuse[object_name] = object_pos_dict
        elif choice == 'forward':
            object_pos_dict = self.pos_dict_for_reuse[object_name]
            target_pose = object_pos_dict['final_pose']
        elif choice == 'reversed':
            object_pos_dict = self.pos_dict_for_reuse[object_name]
            target_pose = object_pos_dict['initial_pose']
        elif choice == 'averaged':
            object_pos_dict = self.pos_dict_for_reuse[object_name]
            target_pose.position.x = (object_pos_dict['initial_pose'].position.x  + object_pos_dict['final_pose'].position.x) / 2
            target_pose.position.y = (object_pos_dict['initial_pose'].position.y  + object_pos_dict['final_pose'].position.y) / 2
            target_pose.position.z = (object_pos_dict['initial_pose'].position.z  + object_pos_dict['final_pose'].position.z) / 2
        else:
            raise NotImplementedError

    waypoints.append(target_pose)
    (plan, fraction) = self.move_group.compute_cartesian_path(
        waypoints, 0.05, 0.0 ,False  # waypoints to follow  # eef_step
    )  # jump_threshold
    self.move_group.execute(plan, wait=True)
'''
},
{# function: (beginning, ending)
"role":"user",
"content": "# define function: (beginning = get_object_center_position('cabinet'), ending = get_object_center_position('handle'))"
},
{
"role": "assistant",
"content":
'''
def figure_direction(beginning: np.array, ending: np.array):
    print("------------------figure_direction------------------------")
    vector = np.array(beginning) - np.array(ending)
    direction = vector / np.linalg.norm(vector)
    print(direction)
    return direction
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

