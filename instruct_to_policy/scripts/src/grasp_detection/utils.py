"""
This file contains utility functions to process and generate grasps:

- Select grasp pose parallel to table surface
- Select grasp pose perpendicular to table surface
- Select grasp pose perpendicular to a predicted axis
- Select grasp pose parallel to an axis
"""

import numpy as np
from typing import List, Tuple, Dict
from numpy.typing import ArrayLike
from scipy.spatial.transform import Rotation as R
from grasp_detection.msg import Grasp
from geometry_msgs.msg import Pose, Point, Quaternion

# gripper local frame definition
DEFAULT_GRIPPER_APPROACH_VECTOR = np.array([0, 0, 1])
DEFAULT_GRIPPER_OPEN_DIRECTION = np.array([0, 1, 0])
DEFAULT_GRIPPER_PLANE_NORMAL = np.array([1, 0, 0])


def argsort_grasp_pose_perpendicular_to_axis(
    grasp_candidates: List[Grasp],
    axis: np.array,
    gripper_approach_vector=DEFAULT_GRIPPER_APPROACH_VECTOR,
) -> List[int]:
    """
    Argsorts a list of grasp candidates based on the dot product between the gripper approach vector and the predicted axis.
    In other words, the gripper approach vector in world frame is perpendicular to the predicted axis.

    Args:
        grasp_candidates (List[Grasp]): List of grasp candidates.
        axis (np.array): Predicted axis.
        gripper_approach_vector (np.array): Approach vector of the gripper in the gripper frame. Defaults to np.array([0, 0, 1]).

    Returns:
        List[int]: List of indices of grasp candidates sorted from most perpendicular to least perpendicular.
    """
    dot_products = []
    for grasp in grasp_candidates:
        gripper_approach_vector_world = (
            R.from_quat(
                [
                    grasp.grasp_pose.orientation.x,
                    grasp.grasp_pose.orientation.y,
                    grasp.grasp_pose.orientation.z,
                    grasp.grasp_pose.orientation.w,
                ]
            ).as_matrix()
            @ gripper_approach_vector
        )
        dot_product = np.dot(gripper_approach_vector_world, axis)
        dot_products.append(dot_product)

    # Calculate the absolute value of the dot product
    dot_products = np.abs(dot_products)
    # The smaller the dot product, the more perpendicular the gripper approach vector is to the predicted axis.
    # Sort in ascending order
    return np.argsort(dot_products)


def argsort_grasp_pose_parallel_to_axis(
    grasp_candidates: List[Grasp],
    axis: np.array,
    gripper_approach_vector=DEFAULT_GRIPPER_APPROACH_VECTOR,
) -> List[int]:
    """
    Argsorts a list of grasp candidates based on the dot product between the gripper approach vector and the predicted axis.
    In other words, the gripper approach vector in world frame is parallel to the predicted axis.

    Args:
        grasp_candidates (List[Grasp]): List of grasp candidates.
        axis (np.array): Predicted axis.
        gripper_approach_vector (np.array): Approach vector of the gripper in the gripper frame. Defaults to np.array([0, 0, 1]).

    Returns:
        List[int]: List of indices of grasp candidates sorted from most parallel to least parallel
    """
    dot_products = []
    for grasp in grasp_candidates:
        gripper_approach_vector_world = (
            R.from_quat(
                [
                    grasp.grasp_pose.orientation.x,
                    grasp.grasp_pose.orientation.y,
                    grasp.grasp_pose.orientation.z,
                    grasp.grasp_pose.orientation.w,
                ]
            ).as_matrix()
            @ gripper_approach_vector
        )
        dot_product = np.dot(gripper_approach_vector_world, axis)
        dot_products.append(dot_product)

    # Calculate the absolute value of the dot product
    dot_products = np.abs(dot_products)
    # The larger the dot product, the more parallel the gripper approach vector is to the predicted axis.
    # Sort in descending order
    return np.argsort(dot_products)[::-1]


def argsort_grasp_pose_perpendicular_to_plane(
    grasp_candidates: List[Grasp],
    plane_normal: np.array,
    gripper_plane_normal=DEFAULT_GRIPPER_PLANE_NORMAL,
) -> List[int]:
    """
    Argsorts a list of grasp candidates based on the dot product between the gripper plane normal and the predicted plane normal.
    In other words, the gripper plane normal in world frame is perpendicular to the predicted plane normal.

    Args:
        grasp_candidates (List[Grasp]): List of grasp candidates.
        plane_normal (np.array): Predicted plane normal.
        gripper_plane_normal (np.array): Normal vector of the gripper plane in the gripper frame. Defaults to DEFAULT_GRIPPER_PLANE_NORMAL.

    Returns:
        List[int]: List of indices of grasp candidates sorted from most perpendicular to least perpendicular
    """
    dot_products = []
    for grasp in grasp_candidates:
        gripper_plane_normal_world = (
            R.from_quat(
                [
                    grasp.grasp_pose.orientation.x,
                    grasp.grasp_pose.orientation.y,
                    grasp.grasp_pose.orientation.z,
                    grasp.grasp_pose.orientation.w,
                ]
            ).as_matrix()
            @ gripper_plane_normal
        )
        dot_product = np.dot(gripper_plane_normal_world, plane_normal)
        dot_products.append(dot_product)

    # Calculate the absolute value of the dot product
    dot_products = np.abs(dot_products)
    # The smaller the dot product, the more perpendicular the gripper plane normal is to the predicted plane normal.
    # Sort in ascending order
    return np.argsort(dot_products)


def argsort_grasp_pose_parallel_to_plane(
    grasp_candidates: List[Grasp],
    plane_normal: np.array,
    gripper_plane_normal=DEFAULT_GRIPPER_PLANE_NORMAL,
) -> List[int]:
    """
    Argsorts a list of grasp candidates based on the dot product between the gripper plane normal and the predicted plane normal.
    In other words, the gripper plane normal in world frame is parallel to the predicted plane normal.

    Args:
        grasp_candidates (List[Grasp]): List of grasp candidates.
        plane_normal (np.array): Predicted plane normal.
        gripper_plane_normal (np.array): Normal vector of the gripper plane in the gripper frame. Defaults to DEFAULT_GRIPPER_PLANE_NORMAL.

    Returns:
        List[int]: List of indices of grasp candidates sorted from most parallel to least parallel
    """
    dot_products = []
    for grasp in grasp_candidates:
        gripper_plane_normal_world = (
            R.from_quat(
                [
                    grasp.grasp_pose.orientation.x,
                    grasp.grasp_pose.orientation.y,
                    grasp.grasp_pose.orientation.z,
                    grasp.grasp_pose.orientation.w,
                ]
            ).as_matrix()
            @ gripper_plane_normal
        )
        dot_product = np.dot(gripper_plane_normal_world, plane_normal)
        dot_products.append(dot_product)

    # Calculate the absolute value of the dot product
    dot_products = np.abs(dot_products)
    # The larger the dot product, the more parallel the gripper plane normal is to the predicted plane normal.
    # Sort in descending order
    return np.argsort(dot_products)[::-1]


def select_grasp_by_preference(
    grasp_candidates: List[Grasp], **kwargs
) -> Tuple[int, ArrayLike]:
    """
    Select the best grasp from a list of grasp candidates based on the given preferences.

    Args:
        grasp_candidates (List[Grasp]): List of grasp candidates.
        preferred_position (np.ndarray): Preferred position of the gripper in world frame.
        preferred_approach_direction (np.ndarray): Preferred approach direction of the gripper in world frame.
        preferred_plane_normal (np.ndarray): Preferred plane normal of the gripper in world frame.
        weight_grasp_score (float): Weight for the grasp score.
        weight_preference_position (float): Weight for the preferred position.
        weight_preference_approach_direction (float): Weight for the preferred approach direction.
        weight_preference_plane_normal (float): Weight for the preferred plane normal.
    """
    preferred_position: np.ndarray = kwargs.get("preferred_position", None)
    preferred_approach_direction: np.ndarray = kwargs.get(
        "preferred_approach_direction", None
    )
    preferred_plane_normal: np.ndarray = kwargs.get("preferred_plane_normal", None)

    weight_grasp_score: float = kwargs.get("weight_grasp_score", 0.01)
    weight_preference_position: float = kwargs.get("weight_distance", 0.2)
    weight_preference_approach_direction: float = kwargs.get(
        "weight_cos_similarity", 0.4
    )
    weight_preference_plane_normal: float = kwargs.get("weight_cos_similarity", 0.4)

    pose_list = [grasp.grasp_pose for grasp in grasp_candidates]
    score_list = [grasp.grasp_score for grasp in grasp_candidates]

    rank_score_list = []

    # get the rank by grasp detection model score
    score_rank_ascend_idx = np.argsort(score_list)
    grasp_rank_score = np.zeros(len(score_list), dtype=int)
    # The score from the rank: worst get 0, best get len(score_list)-1
    grasp_rank_score[score_rank_ascend_idx] = np.arange(len(score_list))
    # rank.append(score_rank)
    rank_score_list.append(grasp_rank_score)

    if preferred_position is not None:
        # if preferred_position is given, choose the grasp pose closest to the preferred position
        position_list = np.array(
            [np.array([p.position.x, p.position.y, p.position.z]) for p in pose_list]
        )
        distance = np.linalg.norm(position_list - preferred_position, axis=1)
        # the larger distance, the smaller score and larger rank
        distance_rank_ascend_idx = np.argsort(distance)
        distance_rank_score = np.zeros(len(distance), dtype=int)
        distance_rank_score[distance_rank_ascend_idx] = np.arange(len(distance))[::-1]
        rank_score_list.append(distance_rank_score)
    else:
        rank_score_list.append(np.zeros(len(score_list), dtype=int))

    if preferred_approach_direction is not None:
        # rank from best to worst
        approach_direction_rank_idx = argsort_grasp_pose_parallel_to_axis(
            grasp_candidates, preferred_approach_direction
        )
        approach_direction_rank_score = np.zeros(
            len(approach_direction_rank_idx), dtype=int
        )
        approach_direction_rank_score[approach_direction_rank_idx] = np.arange(
            len(approach_direction_rank_idx)
        )[::-1]
        rank_score_list.append(approach_direction_rank_score)
    else:
        rank_score_list.append(np.zeros(len(score_list), dtype=int))

    if preferred_plane_normal is not None:
        # rank from best to worst
        plane_normal_rank_idx = argsort_grasp_pose_parallel_to_plane(
            grasp_candidates, preferred_plane_normal
        )
        plane_normal_rank_score = np.zeros(len(plane_normal_rank_idx), dtype=int)
        plane_normal_rank_score[plane_normal_rank_idx] = np.arange(
            len(plane_normal_rank_idx)
        )[::-1]
        rank_score_list.append(plane_normal_rank_score)
    else:
        rank_score_list.append(np.zeros(len(score_list), dtype=int))

    # concatenate weights
    rank_weights = np.array(
        [
            [weight_grasp_score],
            [weight_preference_position],
            [weight_preference_approach_direction],
            [weight_preference_plane_normal],
        ]
    )

    rank_score = np.array(rank_score_list)
    rank_score_weighted_sum = np.sum(rank_score * rank_weights, axis=0)
    best_grasp_idx = np.argmax(rank_score_weighted_sum)

    return best_grasp_idx, rank_score_weighted_sum
