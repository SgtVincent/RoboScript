from typing import Dict, List, Tuple
import numpy as np
import pandas as pd
import rospy
from copy import deepcopy

from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene, ApplyPlanningSceneRequest


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

def matrix2ACMEntries(matrix: np.ndarray) -> List[AllowedCollisionEntry]:
    """
    Convert a matrix of bools to a moveit_msgs/AllowedCollisionMatrix

    :param matrix: np.ndarray of bools
    """

    entries_list = [
        AllowedCollisionEntry(
            enabled=[bool(matrix[i, j]) for j in range(matrix.shape[1])]
        ) for i in range(matrix.shape[0])
    ]
    return entries_list


class CollisionManager:
    """
    This class is used to manage allowed collisions in moveit planning scene.
    
    NOTE: Please check http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/planning_scene_ros_api/planning_scene_ros_api_tutorial.html
    for planning scene ROS API communication details
    
    NOTE: When update default entries, also add entries in the message, since ACM is ONLY updated with msg.entry_names is not empty
    Ref: https://github.com/ros-planning/moveit/blob/984a0ea2a00b4190cad9e08ea0d764c19dbf9975/moveit_core/planning_scene/src/planning_scene.cpp#L1292
    """
    def __init__(self, namespace: str = "", sync_update=True) -> None:
        self.ns = namespace
        self.get_planning_scene = rospy.ServiceProxy(
            f"{self.ns}/get_planning_scene", GetPlanningScene
        )
        self.sync_update = sync_update
        
        self.scene_pub = None
        self.scene_update_proxy = None
        if sync_update:
            self.scene_update_proxy = rospy.ServiceProxy(
                f"{self.ns}/apply_planning_scene", ApplyPlanningScene
            )
        else:
            self.scene_pub = rospy.Publisher(
                f"{self.ns}/move_group/monitored_planning_scene", PlanningScene, queue_size=1
            )

    def get_allowed_collision_matrix(self) -> AllowedCollisionMatrix:
        request = PlanningSceneComponents(
            components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        return self.get_planning_scene(request).scene.allowed_collision_matrix
    
    def get_links(self, acm: AllowedCollisionMatrix) -> Dict[str, int]:
        return {n: i for i, n in enumerate(acm.entry_names)}
    
    def get_link_names(self, name_to_idx_map: Dict[str, int]) -> List[str]:
        return [name for name, index in sorted(name_to_idx_map.items(), key=lambda x: x[1])]

    def get_default_links(self, acm: AllowedCollisionMatrix) -> Dict[str, int]:
        return {n: i for i, n in enumerate(acm.default_entry_names)}

    def update_allowed_collision_matrix(self, acm: AllowedCollisionMatrix) -> None:
        if self.sync_update:
            self.scene_update_proxy(ApplyPlanningSceneRequest(scene=PlanningScene(is_diff=True, allowed_collision_matrix=acm)))
            
            # wait for the update to be done
            rospy.sleep(1)
            _ = self.get_allowed_collision_matrix()
        
        else:
            self.scene_pub.publish(PlanningScene(is_diff=True, allowed_collision_matrix=acm))
            
        rospy.logdebug(f"Moveit Collision Manager: ACM update message sent.")

    def are_allowed(self, link_1: str, link_2: str) -> bool:
        acm = self.get_allowed_collision_matrix()
        name_to_idx_map = self.get_links(acm)

        source_index = name_to_idx_map[link_1]
        target_index = name_to_idx_map[link_2]

        return bool(
            acm.entry_values[source_index].enabled[target_index]
            and acm.entry_values[target_index].enabled[source_index]
        )

    def enable_collisions(self, link_lists:List[Tuple]):
        """
        Enable collisions of a list of links with all other links in the URDF
        """
        acm = self.get_allowed_collision_matrix()

        # update default collision entries 
        expanded_default_entries, default_name_to_idx_map = \
            self._update_default_collision_entries(link_lists, [False] * len(link_lists), acm)

        # check if there are new links in the collision matrix
        old_links = self.get_link_names(self.get_links(acm))
        link_tuple_lists = []
        allowed_lists = []
        for i, link in enumerate(link_lists):
                for old_link in old_links:
                    link_tuple_lists.append((link, old_link))
                    allowed_lists.append(False)
        
        modified_acm = deepcopy(acm)
        # also update collision matrix with
        expanded_matrix_np, name_to_idx_map = self._update_collision_matrix(link_tuple_lists, allowed_lists, acm, default=False)
        
        modified_acm.entry_values = matrix2ACMEntries(expanded_matrix_np)
        modified_acm.entry_names = self.get_link_names(name_to_idx_map)
        modified_acm.default_entry_values = expanded_default_entries
        modified_acm.default_entry_names = self.get_link_names(default_name_to_idx_map)
        
        self.update_allowed_collision_matrix(modified_acm)
        rospy.loginfo(f"Moveit Collision Manager: Enable collisions for {link_lists}")

    def disable_collisions(self, link_lists:List[Tuple]):
        """
        Disable collisions of a list of links with all other links in the URDF
        """
        acm = self.get_allowed_collision_matrix()

        # update default collision entries 
        expanded_default_entries, default_name_to_idx_map = \
            self._update_default_collision_entries(link_lists, [True] * len(link_lists), acm)

        # check if there are new links in the collision matrix
        old_links = self.get_link_names(self.get_links(acm))
        link_tuple_lists = []
        allowed_lists = []
        for i, link in enumerate(link_lists):
                for old_link in old_links:
                    link_tuple_lists.append((link, old_link))
                    allowed_lists.append(True)
        
        modified_acm = deepcopy(acm)
        # also update collision matrix with
        expanded_matrix_np, name_to_idx_map = self._update_collision_matrix(link_tuple_lists, allowed_lists, acm, default=True)
        
        modified_acm.entry_values = matrix2ACMEntries(expanded_matrix_np)
        modified_acm.entry_names = self.get_link_names(name_to_idx_map)
        modified_acm.default_entry_values = expanded_default_entries
        modified_acm.default_entry_names = self.get_link_names(default_name_to_idx_map)
        
        self.update_allowed_collision_matrix(modified_acm)
        rospy.loginfo(f"Moveit Collision Manager: Disable collisions for {link_lists}")

    def set_collision_entries(self, link_tuple_lists: List[Tuple], allowed_lists: List[bool]): 
        """
        Set collisions entries for a list of link tuples in the URDF

        :param link_tuple_lists: [(link_1, link_2), ...] 
        :param allowed_lists: [bool, ...]
        """
        acm = self.get_allowed_collision_matrix()

        expanded_matrix_np, name_to_idx_map = self._update_collision_matrix(link_tuple_lists, allowed_lists, acm)

        # convert expanded matrix to AllowedCollisionMatrix
        modified_acm = deepcopy(acm)
        modified_acm.entry_values = matrix2ACMEntries(expanded_matrix_np)
        modified_acm.entry_names = self.get_link_names(name_to_idx_map)
        
        self.update_allowed_collision_matrix(modified_acm)
        rospy.loginfo(f"Moveit Collision Manager: Set collision entries: {link_tuple_lists} to {allowed_lists}")
        

    def _update_collision_matrix(self, link_tuple_lists: List[Tuple], allowed_lists: List[bool], acm: AllowedCollisionMatrix, default=False):
        
        """
        Update collision matrix with a list of link tuples and allowed values

        :param link_tuple_lists: [(link_1, link_2), ...] 
        :param allowed_lists: [bool, ...]
        :param acm: AllowedCollisionMatrix
        """
        name_to_idx_map = self.get_links(acm)
        
        # check which links are new links and add them to the matrix
        new_links = []
        for link_tuple in link_tuple_lists:
            for link in link_tuple:
                if link not in name_to_idx_map and link not in new_links:
                    new_links.append(link)
                    
        # add new links to name_to_idx_map
        for link in new_links:
            name_to_idx_map[link] = len(name_to_idx_map)
            
        # convert AllowedCollisionMatrix to matrix and expand the matrix with new entries 
        matrix_np = ACM2matrix(acm)
        expanded_matrix_np = (np.ones((len(name_to_idx_map), len(name_to_idx_map))) * default).astype(bool)
        expanded_matrix_np[:matrix_np.shape[0], :matrix_np.shape[1]] = matrix_np

        # assign new entry values to expanded matrix
        for link_tuple, allowed in zip(link_tuple_lists, allowed_lists):
            
            source_index = name_to_idx_map[link_tuple[0]]
            target_index = name_to_idx_map[link_tuple[1]]
            expanded_matrix_np[source_index, target_index] = allowed
            expanded_matrix_np[target_index, source_index] = allowed
            
        return expanded_matrix_np, name_to_idx_map
            
    
    def set_default_collision_entries(self, link_lists: List[str], default_allowed_lists: List[bool]):
        """
        Set default collisions entries for a list of links in the URDF
        
        :param link_lists: [link_1, link_2, ...]
        :param allowed_lists: [bool, ...]
        """
        acm = self.get_allowed_collision_matrix()

        # update default collision entries 
        expanded_default_entries, default_name_to_idx_map = self._update_default_collision_entries(link_lists, default_allowed_lists, acm)

        # check if there are new links in the collision matrix
        old_links = self.get_link_names(self.get_links(acm))
        link_tuple_lists = []
        allowed_lists = []
        for i, link in enumerate(link_lists):
            default_allowed_value = default_allowed_lists[i]
            if link not in old_links:
                for old_link in old_links:
                    link_tuple_lists.append((link, old_link))
                    allowed_lists.append(default_allowed_value)
        
        modified_acm = deepcopy(acm)
        if len(link_tuple_lists) > 0:
            # also update collision matrix with new links if any
            expanded_matrix_np, name_to_idx_map = self._update_collision_matrix(link_tuple_lists, allowed_lists, acm, default=True)
            
            modified_acm.entry_values = matrix2ACMEntries(expanded_matrix_np)
            modified_acm.entry_names = self.get_link_names(name_to_idx_map)
            modified_acm.default_entry_values = expanded_default_entries
            modified_acm.default_entry_names = self.get_link_names(default_name_to_idx_map)
        else:
            modified_acm.default_entry_values = expanded_default_entries
            modified_acm.default_entry_names = self.get_link_names(default_name_to_idx_map)
            
        self.update_allowed_collision_matrix(modified_acm)
        rospy.loginfo(f"Moveit Collision Manager: Set default collision entries: {link_lists} to {allowed_lists}")
        
    def _update_default_collision_entries(self, link_lists: List[str], allowed_lists: List[bool], acm: AllowedCollisionMatrix):
        """
        Update default collision entries with a list of links and allowed values

        :param link_lists: [link_1, link_2, ...]
        :param allowed_lists: [bool, ...]
        :param acm: AllowedCollisionMatrix
        """
        default_name_to_idx_map = self.get_default_links(acm)

        # check which links are new links and add them to the default entries
        new_links = []
        for link in link_lists:
            if link not in default_name_to_idx_map and link not in new_links:
                new_links.append(link)
                
        for link in new_links:
            default_name_to_idx_map[link] = len(default_name_to_idx_map)
            
        # convert default_entry_values to numpy array and expand the matrix with new entries
        default_entries = np.array(acm.default_entry_values, dtype=bool)
        expanded_default_entries = np.zeros(len(default_name_to_idx_map), dtype=bool)
        expanded_default_entries[:default_entries.shape[0]] = default_entries
        
        # assign new entry values to expanded matrix
        for link, allowed in zip(link_lists, allowed_lists):
            index = default_name_to_idx_map[link]
            expanded_default_entries[index] = allowed
            
            
        return expanded_default_entries, default_name_to_idx_map
            
            
    @staticmethod
    def show_matrix(acm: AllowedCollisionMatrix) -> object:
        # name_to_idx_map = {i: n for i, n in enumerate(matrix.entry_names)}
        name_to_idx_map = dict(enumerate(acm.entry_names))
        rows = [["x", *acm.entry_names]]
        for i, row in enumerate(acm.entry_values):
            rows.append([name_to_idx_map[i]] + row.enabled)
        pd.options.display.max_columns = None
        df = pd.DataFrame(rows)
        return df.rename(columns=df.iloc[0])