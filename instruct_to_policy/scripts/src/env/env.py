from typing import List, Tuple, Dict
import numpy as np
import shapely
from shapely.geometry import *
from shapely.affinity import *


class Env:
    """Wrapper for the environment grounding capabilities.
    Currently this class only takes ground truth grounding from simulation,
    but also has the capability to take in predicted grounding from a model in the real world.
    """

    def __init__(self, cfg):
        self._cfg = cfg
        # TODO: get the min and max boundary of table from the sim 
        self._min_xy = np.array(self._cfg["env"]["coords"]["bottom_left"])
        self._max_xy = np.array(self._cfg["env"]["coords"]["top_right"])
        self._table_z = self._cfg["env"]["coords"]["table_z"]
        self._range_xy = self._max_xy - self._min_xy

    def get_obj_name_list(self)-> List[str]:
        raise NotImplementedError("get_obj_name_list() not implemented")

    def get_object_center_position(self, obj_name):
        """ Get object position."""
        raise NotImplementedError("get_object_center_position() not implemented")

    def get_3d_bbox(self, obj_name: str)-> np.ndarray:
        """
        Get the bounding box of the object.
        Args:
            obj_name: name of the object
        Returns:
            bbox: np.ndarray, [x_min, y_min, z_min, x_max, y_max, z_max]
        """
        raise NotImplementedError("get_3d_bbox() not implemented")

    ################# robot contorl interface #################
    def get_end_effector_pose(self):
        """ return robot end-effector pose in robot base frame """
        raise NotImplementedError("get_end_effector_pose() not implemented")

    def get_joint_values(self):
        """ return robot joint values """
        raise NotImplementedError("get_joint_values() not implemented")

    def open_gripper(self):
        """ open gripper """
        raise NotImplementedError("open_gripper() not implemented")
    
    def close_gripper(self):
        """ close gripper """
        raise NotImplementedError("close_gripper() not implemented")
    
    def move_to_pose(self, pose):
        """ move robot end-effector to pose """
        raise NotImplementedError("move_to_pose() not implemented")

    def grasp(self, pose):
        """ grasp object at pose """
        raise NotImplementedError("grasp() not implemented")
    
    def place(self, pose):
        """ place object at pose """
        raise NotImplementedError("place() not implemented")
    
