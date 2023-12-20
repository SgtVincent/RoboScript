from typing import List, Tuple, Dict
import rospy 
import json
import numpy as np 
import re 

from .moveit_gazebo_env import MoveitGazeboEnv
from src.grasp_detection import GraspDetectionBase, GraspDetectionRemote
from src.joint_prediction import JointPredictionGT 


class TrueGroundingEnv(MoveitGazeboEnv):
    """
    Environment to use gazebo GT model state as observation and grounding.
    """
    def __init__(self, cfg) -> None:
        super().__init__(cfg)
        self.use_gt_perception = True
        self.object_metadata_files = cfg["env"]["metadata_files"]
        self.grasp_config = cfg["grasp_detection"]
        self.grasp_method = self.grasp_config["method"] # ["heuristic", "model"]
        
        self.grasp_model = None
        self.groudning_model = None 
        self.joint_prediction_model = None
        
        self.object_info = {}

        self._init_models()
        # self._load_gt_object_info()
    
    def _init_models(self):
        """
        Initialze all models needed for the environment: grounding, grasp detection, etc.
        """
        if self.grasp_method in ["model"]:
            grasp_model_config = self.grasp_config["model_params"]
            self.grasp_model = GraspDetectionRemote(grasp_model_config)
            self.grasp_model.load_model()
            
        # for true grounding env, always use ground truth joint prediction model
        self.joint_prediction_model = JointPredictionGT()
        
    
    def _load_gt_object_info(self):
        """
        Load the ground truth object information from object metadata files 
        TODO: deprecated, remove this function when cleaning up the code
        """
        self.object_info = {}
        for file in self.object_metadata_files:
            with open(file, 'r') as f:
                metadata = json.load(f)
                for k, v in metadata["objects"].items():
                    object_name = v['model_name']
                    self.object_info[object_name] = {
                        'bbox_size': v['bbox_size'],
                        'bbox_center': v['bbox_center'],
                    }
      
      
    def get_obj_name_list(self) -> List[str]:
        """
        Get objects names from gazebo API.
        """
        return self.get_gazebo_model_names()
        
    def get_object_center_position(self, obj_name, **kwargs):
        """
        Get the position of the object in the world frame. 
        This function uses ground truth model state from gazebo and ignore all other parameters.
        """
        # NOTE: get_gt_obj_pose does not work for articulated links, since the mesh might be offset from the link origin
        # gt_pose = self.get_gt_obj_pose(obj_name)
        # if gt_pose is None:
        #     return None
        # return np.array([gt_pose.position.x, gt_pose.position.y, gt_pose.position.z], dtype=float)
        center, size = self.get_gt_bbox(obj_name)
        if center is None or size is None:
            return None
        return np.array(center, dtype=float)
        
    def get_3d_bbox(self, obj_name, **kwargs)->np.array:
        """
        Get the 3D bounding box of the object in the world frame.
        This function uses ground truth model state from gazebo and ignore all other parameters.
        Return [x_min, y_min, z_min, x_max, y_max, z_max]
        """
        center, size = self.get_gt_bbox(obj_name)
        if center is None or size is None:
            return None
        bbox = np.array([center[0] - size[0]/2, center[1] - size[1]/2, center[2] - size[2]/2, 
                            center[0] + size[0]/2, center[1] + size[1]/2, center[2] + size[2]/2])
        return bbox
    
    def get_object_pose(self, obj_name, **kwargs):
        """
        Get the pose of the object in the world frame. 
        This function uses ground truth model state from gazebo and ignore all other parameters.
        """
        gt_pose = self.get_gt_obj_pose(obj_name)
        if gt_pose is None:
            return None
        return gt_pose
    
    
    def detect_objects(self, **kwargs):
        # True grounding env does not need to detect objects since it receives ground truth model state from gazebo
        pass
    
    
    def get_object_joint_axis(self, obj_name: str, position: np.ndarray, type="any")->Dict:
        """
        Get the joint axis closest to the given axis.
        Args:
            obj_name: name of the object
            position: np.ndarray, select the joint closest to this position
            type: str, allowed type of the joint, "any", "revolute", "prismatic"
        Returns:
            closest_axis: Dict, the closest joint axis
                {
                    "joint_position":[
                        0.0,
                        0.0,
                        0.0
                    ],
                    "joint_axis": [
                        -1.0,
                        -8.511809568290118e-08,
                        -1.677630052654422e-07
                    ],
                    "type": "prismatic"
                }
        """
        
        if type == "any":
            joint_types = ["revolute", "prismatic"]
        elif type in ["revolute", "prismatic"]:
            joint_types = [type]
        else:
            raise ValueError("Error in get_object_joint_axis: Invalid type: {}".format(type))
        
        # get joints axes from joint prediction model
        data = {
            "obj_name": obj_name,
            "joint_types": joint_types,
        }
        joints_axes = self.joint_prediction_model.predict(data)
        
        # find the closest joint axis: the distance is between the position (point) and the line (joint axis)
        closest_axis = None
        closest_axis_dist = float('inf')
        for joint_axis in joints_axes:
            joint_position = np.array(joint_axis["joint_position"])
            joint_axis_vector = np.array(joint_axis["joint_axis"])
            
            # compute the distance between the point and the line
            dist = np.linalg.norm(np.cross(joint_axis_vector, position - joint_position)) / np.linalg.norm(joint_axis_vector)
            if dist < closest_axis_dist:
                closest_axis_dist = dist
                closest_axis = joint_axis
    
        return closest_axis        
        