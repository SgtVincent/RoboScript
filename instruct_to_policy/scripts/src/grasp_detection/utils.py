"""
This file contains utility functions to process and generate grasps:

- Select grasp pose parallel to table surface
- Select grasp pose perpendicular to table surface
- Select grasp pose perpendicular to a predicted axis
- Select grasp pose parallel to an axis
"""

import numpy as np 
from typing import List, Tuple, Dict
from scipy.spatial.transform import Rotation as R
from grasp_detection.msg import Grasp
from geometry_msgs.msg import Pose, Point, Quaternion

# gripper local frame definition 
DEFAULT_GRIPPER_APPROACH_VECTOR = np.array([0, 0, 1])
DEFAULT_GRIPPER_OPEN_DIRECTION = np.array([0, 1, 0])
DEFAULT_GRIPPER_PLANE_NORMAL = np.array([1, 0, 0])

def select_grasp_pose_parallel_to_table_surface(grasp_candidates: List[Grasp], table_normal: np.array, 
                                                gripper_plane_normal=DEFAULT_GRIPPER_PLANE_NORMAL)->Grasp:
    """
    Selects a grasp pose parallel to the table surface from a list of grasp candidates.
    In other words, the gripper plane normal in world frame is parallel to the table normal.

    Args:
        grasp_candidates (List[Grasp]): List of grasp candidates.
        table_normal (np.array): Normal vector of the table surface.
        gripper_plane_normal (np.array): Normal vector of the gripper plane in the gripper frame. Defaults to DEFAULT_GRIPPER_PLANE_NORMAL.
    
    Returns:
        Grasp: Selected grasp pose parallel to the table surface.
    """
    selected_grasp = None
    max_dot_product = -np.inf

    for grasp in grasp_candidates:
        gripper_plane_normal_world = np.dot(grasp.pose.orientation, gripper_plane_normal)
        dot_product = np.dot(gripper_plane_normal_world, table_normal)

        if dot_product > max_dot_product:
            max_dot_product = dot_product
            selected_grasp = grasp

    return selected_grasp

def select_grasp_pose_perpendicular_to_table_surface(grasp_candidates: List[Grasp], table_normal: np.array, 
                                                     gripper_plane_normal=DEFAULT_GRIPPER_PLANE_NORMAL)->Grasp:
    """
    Selects a grasp pose perpendicular to the table surface from a list of grasp candidates.
    In other words, the gripper plane normal in world frame is perpendicular to the table normal.

    Args:
        grasp_candidates (List[Grasp]): List of grasp candidates.
        table_normal (np.array): Normal vector of the table surface.
        gripper_plane_normal (np.array): Normal vector of the gripper plane in the gripper frame. Defaults to np.array([1, 0, 0]).

    Returns:
        Grasp: Selected grasp pose perpendicular to the table surface.
    """
    selected_grasp = None
    min_dot_product = np.inf

    for grasp in grasp_candidates:
        gripper_plane_normal_world = np.dot(grasp.pose.orientation, gripper_plane_normal)
        dot_product = np.dot(gripper_plane_normal_world, table_normal)

        if dot_product < min_dot_product:
            min_dot_product = dot_product
            selected_grasp = grasp

    return selected_grasp

def select_grasp_pose_perpendicular_to_axis(grasp_candidates: List[Grasp], axis: np.array, 
                                            gripper_approach_vector=DEFAULT_GRIPPER_APPROACH_VECTOR)-> Grasp:
    """
    Selects a grasp pose perpendicular to a predicted axis from a list of grasp candidates.
    In other words, the gripper approach vector in world frame is perpendicular to the predicted axis.
    
    Args:
        grasp_candidates (List[Grasp]): List of grasp candidates.
        axis (np.array): Predicted axis.
        gripper_approach_vector (np.array): Approach vector of the gripper in the gripper frame. Defaults to np.array([0, 0, 1]).
        
    Returns:
        Grasp: Selected grasp pose perpendicular to the predicted axis.
    """
    selected_grasp = None
    min_dot_product = np.inf

    for grasp in grasp_candidates:
        gripper_approach_vector_world = np.dot(grasp.pose.orientation, gripper_approach_vector)
        dot_product = np.dot(gripper_approach_vector_world, axis)

        if dot_product < min_dot_product:
            min_dot_product = dot_product
            selected_grasp = grasp
            
    return selected_grasp

def select_grasp_pose_parallel_to_axis(grasp_candidates: List[Grasp], axis: np.array, 
                                       gripper_approach_vector=DEFAULT_GRIPPER_APPROACH_VECTOR)-> Grasp:
    """
    Selects a grasp pose parallel to a predicted axis from a list of grasp candidates.
    In other words, the gripper approach vector in world frame is parallel to the predicted axis.
    
    Args:
        grasp_candidates (List[Grasp]): List of grasp candidates.
        axis (np.array): Predicted axis.
        gripper_approach_vector (np.array): Approach vector of the gripper in the gripper frame. Defaults to np.array([0, 0, 1]).
        
    Returns:
        Grasp: Selected grasp pose parallel to the predicted axis.
    """
    selected_grasp = None
    max_dot_product = -np.inf

    for grasp in grasp_candidates:
        gripper_approach_vector_world = np.dot(grasp.pose.orientation, gripper_approach_vector)
        dot_product = np.dot(gripper_approach_vector_world, axis)

        if dot_product > max_dot_product:
            max_dot_product = dot_product
            selected_grasp = grasp
            
    return selected_grasp
