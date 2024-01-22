message_fgen = [
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
    get_obj_name_list,           # Returns a list of names of objects present in the scene
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
You should try your best to use perception and motion planning APIs to define the function.
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
{# generate_arc_path_around_joint(current_pose, joint_axis, joint_position, n, angle)
"role":"user",
"content": "# define function: generate_arc_path_around_joint(current_pose, joint_axis, joint_position, n=20, angle=30)"
},
{
"role":"assistant",
"content": 
'''
def generate_arc_path_around_joint(current_pose, joint_axis, joint_position, n, angle):
    """
    Generate an arc path around a joint in space using ROS geometry_msgs.msg.Pose.

    :param current_pose: geometry_msgs.msg.Pose, the current pose of the end effector
    :param joint_axis: np.ndarray, a 3D unit vector representing the joint's axis
    :param joint_position: np.ndarray, the 3D position of the joint in space
    :param n: int, the number of intermediate poses to generate along the arc
    :param angle: float, the total angle of the arc in degrees
    :return: List[geometry_msgs.msg.Pose], a list of Pose messages representing the arc path
    """
    
    # Convert angle from degrees to radians
    angle_rad = np.deg2rad(angle)
    
    # Calculate the step angle for each intermediate pose
    step_angle = angle_rad / n

    # Generate the arc path
    arc_path = []
    for i in range(n + 1):
        # Calculate the rotation for the current step
        rotation = R.from_rotvec(joint_axis * step_angle * i)
        
        # Calculate the new position by applying the rotation to the vector from the joint to the current position
        position_vector = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z]) - joint_position
        new_position_vector = joint_position + rotation.apply(position_vector)
        
        # Calculate the new orientation by applying the rotation to the current orientation
        current_orientation = R.from_quat([current_pose.orientation.x,
                                           current_pose.orientation.y,
                                           current_pose.orientation.z,
                                           current_pose.orientation.w])
        new_orientation = rotation * current_orientation
        
        # Create a new Pose message for the current step
        new_pose_msg = geometry_msgs.msg.Pose()
        new_pose_msg.position = geometry_msgs.msg.Point(x=new_position_vector[0], y=new_position_vector[1], z=new_position_vector[2])
        new_orientation_quat = new_orientation.as_quat()
        new_pose_msg.orientation = geometry_msgs.msg.Quaternion(x=new_orientation_quat[0],
                                                                y=new_orientation_quat[1],
                                                                z=new_orientation_quat[2],
                                                                w=new_orientation_quat[3])
        arc_path.append(new_pose_msg)
    
    return arc_path
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
{# get_objects_in_receptacle(receptacle_name)
"role": "user",
"content": "# define function: get_objects_in_receptacle(receptacle_name)"
},
{
"role": "assistant",
"content": '''
def get_objects_in_receptacle(receptacle_name):
    # Use the bbox to check if the object is inside the receptacle
    objects = get_obj_name_list()
    objects_in_receptacle = []
    
    # get the bbox of the receptacle
    bbox_receptacle = get_3d_bbox(receptacle_name)
    for obj in objects:
        if obj != receptacle_name:
            # check if the object is inside the receptacle
            obj_center = get_object_center_position(obj)
            if_obj_in_receptacle_x = bbox_receptacle[0] < obj_center[0] < bbox_receptacle[3]
            if_obj_in_receptacle_y = bbox_receptacle[1] < obj_center[1] < bbox_receptacle[4]
            if_obj_in_receptacle_z = bbox_receptacle[2] < obj_center[2] < bbox_receptacle[5]
            if if_obj_in_receptacle_x and if_obj_in_receptacle_y and if_obj_in_receptacle_z:
                objects_in_receptacle.append(obj)
    
    return objects_in_receptacle
'''
},
{# check_same_orientation(object_name_1, object_name_2)
"role": "user",
"content": "# define function: check_same_orientation(object_name_1='square_plate', object_name_2='blue_bowl')"
},
{
"role": "assistant",
"content": '''
def check_same_orientation(object_name_1, object_name_2):
    # Use the pose to check if the two objects have same orientation
    pose_1 = get_object_pose(object_name_1)
    pose_2 = get_object_pose(object_name_2)
    orientation_1 = np.array([pose_1.orientation.x, pose_1.orientation.y, pose_1.orientation.z, pose_1.orientation.w])
    orientation_2 = np.array([pose_2.orientation.x, pose_2.orientation.y, pose_2.orientation.z, pose_2.orientation.w])
    return np.allclose(orientation_1, orientation_2)
'''
},
]

