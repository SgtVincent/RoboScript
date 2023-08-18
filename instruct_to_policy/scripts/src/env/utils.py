from typing import List, Optional, Union, Tuple
import os
import xml.etree.ElementTree as ElementTree
from moveit_commander import PlanningSceneInterface
import moveit_commander

import rospy
import rospkg
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.srv import (
    GetModelState,
    GetWorldProperties,
    GetModelProperties,
)

from src.env.gazebo_env import GazeboEnv

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

