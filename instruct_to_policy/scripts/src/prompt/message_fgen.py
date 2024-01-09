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
    get_plane_normal,            # Get the plane normal of an object closest to the given position. Args: obj_name: name of the object position: np.ndarray, select the plane closest to this position. Returns: np.ndarray, normal vector [x,y,z]
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
    move_in_direction, # Move the gripper in the given direction in a straight line by certain distance. Note that you can use the surface normal and the joint axis. Args: axis: np.array, the move direction vector ; distance: float.
    generate_arc_path_around_joint, # Generate a rotational gripper path of poses around the revolute joint. Args: current_pose: Pose, current pose of the gripper; joint_axis: np.array, the joint axis of the revolute joint; joint_position: np.array, the joint position of the revolute joint; n: int, number of waypoints; angle: float, angle of rotation in degree. Returns: path: List[Pose]
    follow_path,     # Move the gripper to follow a path of poses. Args: path: List[Pose]
    get_gripper_pose, # Get the gripper pose. No args. Returns: pose: Pose
    grasp,           # Executes a grasp motion at the grasp_pose. Args: grasp_pose: Pose
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
    Generate a rotational gripper path of poses around the revolute joint.

    :param current_pose: Pose, the current pose of the end effector
    :param joint_axis: np.ndarray, a 3D unit vector representing the joint's axis
    :param joint_position: np.ndarray, the 3D position of the joint in space
    :param n: int, the number of intermediate poses to generate along the arc
    :param angle: float, the total angle of the arc in degrees
    :return: List[Pose], a list of Pose messages representing the arc path
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
        new_pose_msg = Pose()
        new_pose_msg.position = Point(x=new_position_vector[0], y=new_position_vector[1], z=new_position_vector[2])
        new_orientation_quat = new_orientation.as_quat()
        new_pose_msg.orientation = Quaternion(x=new_orientation_quat[0],
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
    target_pose.position.x = normalized_axis[0] * distance + current_pose.position.x
    target_pose.position.y = normalized_axis[1] * distance + current_pose.position.y
    target_pose.position.z = normalized_axis[2] * distance + current_pose.position.z
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
            if bbox_receptacle[0] < obj_center[0] < bbox_receptacle[3] and bbox_receptacle[1] < obj_center[1] < bbox_receptacle[4] and bbox_receptacle[2] < obj_center[2] < bbox_receptacle[5]:
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
{# check_is_lying(object_name)
"role": "user",
"content": "# define function: check_is_lying(object_name='bottle')"
},
{
"role": "assistant",
"content": '''
def is_lying(object_name):
    # Judge if the object is lying by its orientation:
    # Calculate the angle between the object's z-axis and the table normal
    angle_threshold = np.radians(60) # 60 degree
    table_normal = np.array([0, 0, 1])
    
    # Compute the angle between the object's z-axis and the table normal
    object_gt_pose = get_object_pose(object_name)
    object_rotation_matrix = R.from_quat([object_gt_pose.orientation.x, object_gt_pose.orientation.y, object_gt_pose.orientation.z, object_gt_pose.orientation.w]).as_matrix()
    z_axis = np.array([0, 0, 1])
    object_z_axis = object_rotation_matrix @ z_axis
    angle = np.arccos(np.clip(np.dot(object_z_axis, table_normal), -1, 1))

    # If the angle is larger than the threshold, the object is lying
    return angle > angle_threshold
'''
}
]

