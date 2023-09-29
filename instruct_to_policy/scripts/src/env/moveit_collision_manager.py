from typing import Dict, List, Tuple
import numpy as np
import pandas as pd
import rospy
from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene


def ACM2matrix(acm: AllowedCollisionMatrix) -> np.ndarray:
    """
    Convert a moveit_msgs/AllowedCollisionMatrix to a matrix of bools

    :param acm: moveit_msgs/AllowedCollisionMatrix
    :return: np.ndarray of bools
    """
    return np.array([
        [entry.enabled[i] for i in range(len(entry.enabled))]
        for entry in acm.entry_values
    ], dtype=bool)

def matrix2ACM(matrix: np.ndarray, names: List[str]) -> AllowedCollisionMatrix:
    """
    Convert a matrix of bools to a moveit_msgs/AllowedCollisionMatrix

    :param matrix: np.ndarray of bools
    :param names: List of names for each row/column
    :return: moveit_msgs/AllowedCollisionMatrix
    """
    assert matrix.shape[0] == len(names)
    acm = AllowedCollisionMatrix()
    acm.entry_names = names
    acm.entry_values = [
        AllowedCollisionEntry(
            enabled=[bool(matrix[i, j]) for j in range(matrix.shape[1])]
        ) for i in range(matrix.shape[0])
    ]
    return acm

class CollisionManager:
    def __init__(self, namespace: str = "") -> None:
        self.ns = namespace
        self.get_planning_scene = rospy.ServiceProxy(
            f"{self.ns}/get_planning_scene", GetPlanningScene
        )
        self.scene = rospy.Publisher(
            f"{self.ns}/move_group/monitored_planning_scene", PlanningScene, queue_size=0
        )

    def get_collision_matrix(self) -> AllowedCollisionMatrix:
        request = PlanningSceneComponents(
            components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        return self.get_planning_scene(request).scene.allowed_collision_matrix

    def get_links(self, matrix: AllowedCollisionMatrix) -> Dict[str, int]:
        return {n: i for i, n in enumerate(matrix.entry_names)}
    
    def get_link_names(self, name_map: Dict[str, int]) -> List[str]:
        return [name for name, index in sorted(name_map.items(), key=lambda x: x[1])]

    def update_matrix(self, matrix: AllowedCollisionMatrix) -> None:
        self.scene.publish(PlanningScene(is_diff=True, allowed_collision_matrix=matrix))

    def are_allowed(self, link_1: str, link_2: str) -> bool:
        matrix = self.get_collision_matrix()
        name_map = self.get_links(matrix)

        source_index = name_map[link_1]
        target_index = name_map[link_2]

        return bool(
            matrix.entry_values[source_index].enabled[target_index]
            and matrix.entry_values[target_index].enabled[source_index]
        )

    def set_collision_entries(self, link_tuple_lists: List[Tuple], allowed_lists: List[bool]): 
        """
        Set collisions entries for a list of link tuples in the URDF

        :param link_tuple_lists: [(link_1, link_2), ...] 
        :param allowed_lists: [bool, ...]
        """
        matrix = self.get_collision_matrix()
        name_map = self.get_links(matrix)
        
        # check which links are new links and add them to the matrix
        new_links = []
        for link_tuple in link_tuple_lists:
            for link in link_tuple:
                if link not in name_map and link not in new_links:
                    new_links.append(link)
                    
        # add new links to name_map
        for link in new_links:
            name_map[link] = len(name_map)
            
        # convert AllowedCollisionMatrix to matrix and expand the matrix with new entries 
        matrix_np = ACM2matrix(matrix)
        expanded_matrix_np = np.zeros((len(name_map), len(name_map)), dtype=bool) # default to False
        expanded_matrix_np[:matrix_np.shape[0], :matrix_np.shape[1]] = matrix_np

        # assign new entry values to expanded matrix
        for link_tuple, allowed in zip(link_tuple_lists, allowed_lists):
            
            source_index = name_map[link_tuple[0]]
            target_index = name_map[link_tuple[1]]
            expanded_matrix_np[source_index, target_index] = allowed
            expanded_matrix_np[target_index, source_index] = allowed
            
        # convert expanded matrix to AllowedCollisionMatrix
        expanded_matrix = matrix2ACM(expanded_matrix_np, self.get_link_names(name_map))
        
        self.update_matrix(expanded_matrix)


    @staticmethod
    def show_matrix(matrix: AllowedCollisionMatrix) -> object:
        # name_map = {i: n for i, n in enumerate(matrix.entry_names)}
        name_map = dict(enumerate(matrix.entry_names))
        rows = [["x", *matrix.entry_names]]
        for i, row in enumerate(matrix.entry_values):
            rows.append([name_map[i]] + row.enabled)
        pd.options.display.max_columns = None
        df = pd.DataFrame(rows)
        return df.rename(columns=df.iloc[0])