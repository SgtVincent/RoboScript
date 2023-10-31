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

    def is_obj_visible(self, obj_name):
        return obj_name in self.object_names

    def get_obj_names(self)-> List[str]:
        raise NotImplementedError("get_obj_names() not implemented")

    def denormalize_xy(self, pos_normalized):
        return pos_normalized * self._range_xy + self._min_xy

    def get_corner_positions(self):
        unit_square = box(0, 0, 1, 1)
        normalized_corners = np.array(list(unit_square.exterior.coords))[:4]
        corners = np.array(([self.denormalize_xy(corner) for corner in normalized_corners]))
        return corners

    def get_side_positions(self):
        side_xs = np.array([0, 0.5, 0.5, 1])
        side_ys = np.array([0.5, 0, 1, 0.5])
        normalized_side_positions = np.c_[side_xs, side_ys]
        side_positions = np.array(
            ([self.denormalize_xy(corner) for corner in normalized_side_positions])
        )
        return side_positions

    def get_obj_pos(self, obj_name):
        """ Get object position."""
        raise NotImplementedError("get_obj_pos() not implemented")

    def get_bbox(self, obj_name):
        """
        Get the bounding box of the object.
        """
        raise NotImplementedError("get_bbox() not implemented")

    def get_mesh(self, obj_name):
        """
        Get the colllision mesh of the object.
        """
        raise NotImplementedError("get_mesh() not implemented")

    def get_object_collision(self, obj_name):
        """
        Get the collision mesh/ bounding box of the object.
        """
        raise NotImplementedError("get_object_collision() not implemented")

    def get_color(self, obj_name):
        raise NotImplementedError("get_color() not implemented")

    def get_corner_positions(self):
        normalized_corners = np.array([[0, 1], [1, 1], [0, 0], [1, 0]])
        return np.array(([self.denormalize_xy(corner) for corner in normalized_corners]))

    def get_side_positions(self):
        normalized_sides = np.array([[0.5, 1], [1, 0.5], [0.5, 0], [0, 0.5]])
        return np.array(([self.denormalize_xy(side) for side in normalized_sides]))

    def get_corner_name(self, pos):
        corner_positions = self.get_corner_positions()
        corner_idx = np.argmin(np.linalg.norm(corner_positions - pos, axis=1))
        return ["top left corner", "top right corner", "bottom left corner", "botom right corner"][
            corner_idx
        ]

    def get_side_name(self, pos):
        side_positions = self.get_side_positions()
        side_idx = np.argmin(np.linalg.norm(side_positions - pos, axis=1))
        return ["top side", "right side", "bottom side", "left side"][side_idx]

    ################# robot contorl interface #################
    def get_ee_pose(self):
        """ return robot end-effector pose in robot base frame """
        raise NotImplementedError("get_ee_pose() not implemented")

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

    def move_joints_to(self, joint_values):
        """ move robot joints to joint_values """
        raise NotImplementedError("move_joints_to() not implemented")

    def grasp(self, pose):
        """ grasp object at pose """
        raise NotImplementedError("grasp() not implemented")
    
    def place(self, pose):
        """ place object at pose """
        raise NotImplementedError("place() not implemented")
    
