from typing import List, Tuple, Dict
import numpy as np
from numpy.typing import ArrayLike
import shapely
from shapely.geometry import *
from shapely.affinity import *
from geometry_msgs.msg import Quaternion, Pose, Point
from grasp_detection.msg import Grasp 
from src.grasp_detection.utils import select_grasp_by_preference

class Env:
    """Wrapper for the environment grounding capabilities.
    The perception interface and robot control interface should not be implemented here.
    They should be implemented by subclasses depending on the specific environment.
    e.g. 
    - Gazebo + Ground Truth Perception + Moveit + Franka Robot
    - Real World + Perception pipeline + Moveit + UR5 Robot
    Other environment-irrelevant  mid-level tools should be provided here to share across different environments.
    """

    def __init__(self, cfg):
        self._cfg = cfg
        
        self.grasp_model = None
        self.grounding_model = None
        self.joint_prediction_model = None

    def get_object_name_list(self)-> List[str]:
        raise NotImplementedError("get_object_name_list() not implemented")

    ################# robot contorl interface #################
    def get_gripper_pose(self)->Pose:
        """ return robot gripper pose in robot base frame """
        raise NotImplementedError("get_gripper_pose() not implemented")

    def get_joint_values(self):
        """ return robot joint values """
        raise NotImplementedError("get_joint_values() not implemented")

    def get_robot_base_position(self):
        """Get the position of the robot base.""" 
        raise NotImplementedError("get_robot_base_position() not implemented")

    def open_gripper(self):
        """ open gripper """
        raise NotImplementedError("open_gripper() not implemented")
    
    def close_gripper(self):
        """ close gripper """
        raise NotImplementedError("close_gripper() not implemented")
    
    def move_to_pose(self, pose):
        """ move robot end-effector to pose """
        raise NotImplementedError("move_to_pose() not implemented")

    def follow_path(self, path):
        """ follow a path """
        raise NotImplementedError("follow_path() not implemented")

    def grasp(self, pose):
        """ grasp object at pose """
        raise NotImplementedError("grasp() not implemented")
    
    def place(self, pose):
        """ place object at pose """
        raise NotImplementedError("place() not implemented")
    
    ################ perception interface ################
    def get_sensor_data(self)->Dict:
        """ get sensor data from perception model """
        raise NotImplementedError("get_sensor_data() not implemented")
    
    def detect_objects(self, *nargs, **kwargs):
        """ detect objects in the scene with perception model"""
        raise NotImplementedError("detect_objects() not implemented")
    
    def get_object_center_position(self, object_name)->ArrayLike:
        """ Get object position."""
        raise NotImplementedError("get_object_center_position() not implemented")

    def get_3d_bbox(self, object_name: str)-> ArrayLike:
        """
        Get the bounding box of the object.
        Args:
            object_name: name of the object
        Returns:
            bbox: np.ndarray, [x_min, y_min, z_min, x_max, y_max, z_max]
        """
        raise NotImplementedError("get_3d_bbox() not implemented")
    
    def get_object_joints_axes(self, object_name: str)->List:
        """
        Get the joint axes for the given list of joint names.
        Args:
            joint_names: List of joint names.
        Returns:
            joints_axes: List of joint axes.
        """
        raise NotImplementedError("get_joint_axes() not implemented")
    
    def get_object_joint_info(self, object_name: str, position: np.ndarray, type="any")->Dict:
        """
        Get the joint axis closest to the given axis.
        Args:
            object_name: name of the object
            position: np.ndarray, select the joint closest to this position
            type: str, allowed type of the joint, "any", "revolute", "prismatic"
        Returns:
            closest_axis: the closest joint axis
        """
        raise NotImplementedError("get_object_joint_info() not implemented")
    
    def get_plane_normal(self, object_name: str, position: np.ndarray)->np.ndarray:
        """
        Get the plane normal of an object closest to the given position.
        Args:
            object_name: name of the object
            position: np.ndarray, select the plane closest to this position
        Returns:
            closest_normal: np.ndarray the closest plane normal
        """
        raise NotImplementedError("get_plane_normal() not implemented")
    
    ################ multimodal grounding interface ################
    def parse_question(self, question, **kwargs):
        """ parse question into a dictionary """
        raise NotImplementedError("parse_question() not implemented")
    
    ################ parse pose tools ##############################
    def parse_place_pose(self, object_name, receptacle_name:str=None, **kwargs)->Pose:
        """
        Parse place pose for the object. 
        """
        raise NotImplementedError("parse_place_pose() not implemented")
    
    def parse_adaptive_shape_grasp_pose(self, object_name, **kwargs)->Pose:
        """
        Parse grasp pose for the object. Use ground truth grounding and grasp detection model.
        Args:
            object_name: name of the object
            preferred_position: Optional(np.ndarray), prefered gripper tip point position 
            preferred_approach_direction: Optional(np.ndarray), prefered gripper approach direction
            preferred_plane_normal: Optional(np.ndarray), prefered gripper plane normal direction  
            description: str, description of the pose
        """
        object_bbox = self.get_3d_bbox(object_name)    
        description:str = kwargs.get('description', None)
        
        # get visual input from perception model
        sensor_data = self.get_sensor_data()
        
        # add 2d/3d bounding boxes to sensor data
        # NOTE: currently only support one object 
        # object_2d_bbox_list = self.scene.get_object_2d_bbox_list(object_name)
        # detections_list = []
        # for bbox in object_2d_bbox_list:
        #     if len(bbox) > 0:
        #         detections_list.append({object: bbox})
                
        bbox_center = (object_bbox[:3] + object_bbox[3:]) / 2
        bbox_size = object_bbox[3:] - object_bbox[:3]
                
        data = {
            # 'detections_list': detections_list,
            'bboxes_3d_dict':{
                object_name:{'center': bbox_center, 'size': bbox_size}
            }
        }
        data.update(sensor_data)
        
        # call grasp detection service
        grasp_candidates: List[Grasp] = self.grasp_model.predict(data)

        # select grasp by preference
        best_grasp_idx, _ = select_grasp_by_preference(grasp_candidates, **kwargs)
        best_grasp_pose = grasp_candidates[best_grasp_idx].grasp_pose
        
        return best_grasp_pose

    def parse_central_lift_grasp_pose(self, object_name, **kwargs)->Pose:
        """
        Parse central lift grasp pose for the object. Use ground truth grounding and heuristic place position calculation.
        """
        description = kwargs.get('description', "top")
        
        object_bbox = self.get_3d_bbox(object_name)
        object_center = (object_bbox[:3] + object_bbox[3:]) / 2
        
        pose = Pose()
        pre_defined_depth = 0.03 # for franka hand
        if description == "top":
            pose.position = Point(object_center[0], object_center[1], object_bbox[5] - pre_defined_depth)
        elif description == "center":
            pose.position = Point(object_center[0], object_center[1], object_center[2])
        else:
            # by default grasp top 
            pose.position = Point(object_center[0], object_center[1], object_bbox[5])
        pose.orientation = Quaternion(-1.0,0.0,0.0,0.0)
        return pose

    def parse_horizontal_grasp_pose(self, object_name, **kwargs)->Pose:
        """ 
        Parse horizontal pose for grasping drawer handle. (master pose for cabinet drawer pulling)
        Currently the position of the handle if the GT bounding box center of handle in gazebo. 
        The gripper is horizontal and perpendicular to the handle.
        """
        pre_defined_horizontal_orientation = Quaternion(-0.5, -0.5, 0.5, 0.5) # for franka hand 
    
        handle_bbox = self.get_3d_bbox(object_name)
        handle_center = (handle_bbox[:3] + handle_bbox[3:]) / 2
        pose = Pose()
        pose.position = Point(*handle_center)
        pose.orientation = pre_defined_horizontal_orientation 
        return pose