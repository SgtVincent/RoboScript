from typing import List, Optional, Union, Tuple
import numpy as np
import numpy.random as random
from scipy.spatial.transform import Rotation as R

import xml.etree.ElementTree as ElementTree
from moveit_commander import PlanningSceneInterface

import rospy
import rospkg
from geometry_msgs.msg import Pose, PoseStamped


################## ROS utils ###################


def get_pose_msg(position: List, orientation: List) -> Pose:
    pose_msg = Pose()
    pose_msg.position.x = position[0]
    pose_msg.position.y = position[1]
    pose_msg.position.z = position[2]
    pose_msg.orientation.x = orientation[0]
    pose_msg.orientation.y = orientation[1]
    pose_msg.orientation.z = orientation[2]
    pose_msg.orientation.w = orientation[3]
    return pose_msg


def get_stamped_pose(
    position: List, orientation: List, frame_id="world"
) -> PoseStamped:
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.pose = get_pose_msg(position, orientation)
    return pose_stamped


def pose_to_list(pose: Union[Pose, PoseStamped]) -> Tuple[List, List]:
    if isinstance(pose, PoseStamped):
        pose = pose.pose
    return [pose.position.x, pose.position.y, pose.position.z], [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]

def pose_msg_to_matrix(pose: Pose) -> np.ndarray:
    """
    Convert a pose message to a 4x4 transformation matrix
    """
    position = np.array([pose.position.x, pose.position.y, pose.position.z])
    orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    
    mat = np.eye(4)
    mat[:3, 3] = position
    mat[:3, :3] = R.from_quat(orientation).as_matrix()
    return mat
    
################### 3D geometry utils ###################

def get_axis_aligned_bbox(bbox_center, bbox_size, transform):
    """
    Returns the axis-aligned bounding box of the object given the object's center and size
    @param bbox_center: center of the 3D bounding box
    @param bbox_size: size of the 3D bounding box
    @transform: (4,4) numpy matrix representing the transformation matrix of the object
    # 1. get bbox corner points in the object frame
    # 2. transform the bbox corner points to the world frame
    # 3. get the axis-aligned bounding box in the world frame from the transformed bbox corner points
    """

    bbox_corner_points = np.array([
        [-bbox_size[0]/2, -bbox_size[1]/2, -bbox_size[2]/2],
        [-bbox_size[0]/2, -bbox_size[1]/2, bbox_size[2]/2],
        [-bbox_size[0]/2, bbox_size[1]/2, -bbox_size[2]/2],
        [-bbox_size[0]/2, bbox_size[1]/2, bbox_size[2]/2],
        [bbox_size[0]/2, -bbox_size[1]/2, -bbox_size[2]/2],
        [bbox_size[0]/2, -bbox_size[1]/2, bbox_size[2]/2],
        [bbox_size[0]/2, bbox_size[1]/2, -bbox_size[2]/2],
        [bbox_size[0]/2, bbox_size[1]/2, bbox_size[2]/2],
    ])
    bbox_corner_points = bbox_corner_points + bbox_center
    bbox_corner_points = bbox_corner_points.T
    bbox_corner_points = np.vstack((bbox_corner_points, np.ones((1,8))))
    bbox_corner_points = np.dot(transform, bbox_corner_points)
    bbox_corner_points = bbox_corner_points[:3, :].T
    bbox_min = np.min(bbox_corner_points, axis=0)
    bbox_max = np.max(bbox_corner_points, axis=0)
    aa_bbox_center = (bbox_min + bbox_max) / 2
    aa_bbox_size = bbox_max - bbox_min

    return aa_bbox_center, aa_bbox_size


def is_collision(bbox1, bbox2):
    # Check for overlap in x, y, and z axes
    overlap_x = not (bbox1[0] > bbox2[3] or bbox1[3] < bbox2[0])
    overlap_y = not (bbox1[1] > bbox2[4] or bbox1[4] < bbox2[1])
    overlap_z = not (bbox1[2] > bbox2[5] or bbox1[5] < bbox2[2])

    # If overlap in all three axes, then the bounding boxes collide
    return overlap_x and overlap_y and overlap_z

def calculate_place_position(object_bbox, receptacle_bbox, obstacle_bbox_list, max_tries=100):
    """
    Given object bbox, receptacle bbox, and other obstacle objects bboxes, calculate a valid position to place the object.
    The position is calculated as the center of the object bounding box above the receptacle bounding box.
    Each bounding box is [[x_min, y_min, z_min], [x_max, y_max, z_max]]
    """
    # Calculate boundaries for x and y within the receptacle
    x_min = receptacle_bbox[0] + (receptacle_bbox[3] - receptacle_bbox[0]) * 0.25
    x_max = receptacle_bbox[0] + (receptacle_bbox[3] - receptacle_bbox[0]) * 0.75
    y_min = receptacle_bbox[1] + (receptacle_bbox[4] - receptacle_bbox[1]) * 0.25
    y_max = receptacle_bbox[1] + (receptacle_bbox[4] - receptacle_bbox[1]) * 0.75
    
    # Calculate the object bounding box size
    object_size = np.array([object_bbox[3] - object_bbox[0], object_bbox[4] - object_bbox[1], object_bbox[5] - object_bbox[2]])
    
    # Calculate the z position which is constant across all placement attempts
    z = receptacle_bbox[5] + object_size[2] / 2
     
    # Initialize the object position
    position = np.array([0, 0, z])
    
    # Try to find a valid position
    for _ in range(max_tries):
        # Randomly sample a new position
        position[0] = random.uniform(x_min, x_max)
        position[1] = random.uniform(y_min, y_max)
        
        # Create a new bounding box for the object in the new position
        new_object_bbox = np.array([position[0] - object_size[0] / 2, position[1] - object_size[1] / 2, position[2] - object_size[2] / 2, 
                                    position[0] + object_size[0] / 2, position[1] + object_size[1] / 2, position[2] + object_size[2] / 2])
        
        # If the object is in collision with any other object, continue to the next attempt
        if any(is_collision(new_object_bbox, obstacle_bbox) for obstacle_bbox in obstacle_bbox_list):
            continue
        
        # If no collision, return the found position
        return position
    
    # return last trial even if it is in collision
    return position

#################### Gazebo and MoveIt utils ####################


def load_model_into_moveit(
    sdf_path,
    pose: Pose,
    scene: PlanningSceneInterface,
    model_name="model",
    link_name="link",
    ref_frame="world",
):
    # try:
    shape_idx = 0
    # convert pose to pose stamped
    pose_stamped = get_stamped_pose(
        [pose.position.x, pose.position.y, pose.position.z],
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
        ref_frame,
    )
    #
    if model_name == "ground_plane":
        scene.add_plane(f"{model_name}_{shape_idx}_plane", pose_stamped)
        return

    # parse model from sdf file
    # TODO: create class to maintain all loaded models in memory
    tree = ElementTree.parse(sdf_path)
    root = tree.getroot()

    # NOTE: Assume there is only one link for the object
    # Iterate over models and collision geometries
    link = root.find(f".//link[@name='{link_name}']")
    for collision in link.findall(".//collision"):
        collision_geometry = collision.find(".//geometry")

        # Iterate over all shapes in the collision geometry
        for shape in collision_geometry:
            # Add collision object to moveit planning scene by shape type

            if shape.tag == "box":
                size = [float(value) for value in shape.find("size").text.split()]
                pose = link.find(".//pose").text  # Adjust pose if needed
                scene.add_box(f"{model_name}_{shape_idx}_box", pose_stamped, size)
                shape_idx += 1

            elif shape.tag in "sphere":
                radius = float(shape.find("radius").text)
                scene.add_sphere(
                    f"{model_name}_{shape_idx}_sphere", pose_stamped, radius
                )
                shape_idx += 1

            elif shape.tag == "cylinder":
                radius = float(shape.find("radius").text)
                length = float(shape.find("length").text)
                scene.add_cylinder(
                    f"{model_name}_{shape_idx}_cylinder", pose_stamped, length, radius
                )
                shape_idx += 1

            elif shape.tag == "cone":
                radius = float(shape.find("radius").text)
                length = float(shape.find("length").text)
                scene.add_cone(
                    f"{model_name}_{shape_idx}_cone", pose_stamped, length, radius
                )
                shape_idx += 1

            elif shape.tag == "plane":
                normal = [float(value) for value in shape.find("normal").text.split()]
                # size not meaningful for moveit
                scene.add_plane(f"{model_name}_{shape_idx}_plane", pose_stamped, normal)
                shape_idx += 1

            elif shape.tag == "mesh":
                # raise NotImplementedError("Meshes are not supported yet")
                rospy.logwarn(f"Mesh loading not supported yet, coming soon...")
            else:
                rospy.logwarn(f"SolidPrimitive {shape.tag} type not supported")

    # except Exception as e:
    #     return e
