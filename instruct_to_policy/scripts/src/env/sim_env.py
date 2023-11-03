from typing import List, Tuple, Dict
import rospy 
import json
import numpy as np 
from geometry_msgs.msg import Quaternion, Pose, Point

from grasp_detection.msg import Grasp as GraspMsg
from .moveit_gazebo_env import MoveitGazeboEnv
from src.grasp_detection import GraspDetectionBase, GraspDetectionRemote
from src.grounding_model import GroundingBase, GroundingEmbodiedGPT
# from src.grasp_detection.utils import Grasp
from src.env.utils import calculate_place_position
from src.perception.scene_manager import SceneManager

class SimEnv(MoveitGazeboEnv):
    """
    Simple grounding environment to use gazebo GT model state as observation, and GIGA for grasp pose prediction.
    """
    def __init__(self, cfg) -> None:
        super().__init__(cfg)
        self.object_metadata_files = cfg["env"]["metadata_files"]
        self.grasp_config = cfg["grasp_detection"]
        self.grasp_method = self.grasp_config["method"] # ["heuristic", "model"]
        
        self.grounding_config = cfg["grounding_model"]
        
        self.grasp_model = None
        self.groudning_model = None
        
        # scene manager to manage the scene objects, 3D reconstruction, detections, etc.
        self.scene = SceneManager()

        self._init_models()
    
    def _init_models(self):
        """
        Initialze all models needed for the environment: grounding, grasp detection, etc.
        """
        if self.grasp_method in ["model"]:
            grasp_model_config = self.grasp_config["model_params"]
            self.grasp_model = GraspDetectionRemote(grasp_model_config)
            self.grasp_model.load_model()
        
        self.grounding_model = GroundingEmbodiedGPT(self.grounding_config)


    def get_object_center_position(self, obj_name, **kwargs):
        """
        Get the position of the object in the world frame. 
        This function uses ground truth model state from gazebo and ignore all other parameters.
        """
        # TODO: Get the position of the object from perception model or find another way to get the position
        pass

        
    def get_3d_bbox(self, obj_name, **kwargs)->np.array:
        """
        Get the bounding box of the object.
        Args:
            obj_name: name of the object
        Returns:
            bbox: np.ndarray, [x_min, y_min, z_min, x_max, y_max, z_max]
        """
        # TODO: Get the bounding box of the object from perception model 
        pass
    
    def parse_grasp_pose(self, object_name, description="flexible_mode"):
        """
        object_name --[MM]--> bbox_2d_list --[anygrasp]--> pose_list --> pose 
        """
        bbox_2d_list = self.groudning_model.get_3d_bbox(object_name)
        
        sensor_data = self.get_sensor_data()   
                 
        data = {
            'depth_bboxes': bbox_2d_list
        }
        data.update(sensor_data)
        
        # call grasp detection service
        pose_list, width_list, score_list = self.grasp_model.predict(data)
        
        # sort by score and get the best grasp
        best_grasp_idx = np.argmax(score_list)
        pose = pose_list[best_grasp_idx]
        
        return pose

    def parse_place_pose(self, object_name, **kwargs):
        """
        object_name --[MM]--> bbox_2d_list --[3D fusion] --> object_3d_bbox
        receptable_name --[MM]--> bbox_2d_list --[3D fusion] --> receptacle_3d_bbox
        object_3d_bbox, receptacle_3d_bbox --[place_model]--> pose_list --> pose
        """
        # get parameters from kwargs
        receptacle_name: str = kwargs.get('receptacle_name', None)
        position: np.ndarray = kwargs.get('position', None)
        description: str= kwargs.get('description', "gripper current pose") 
        # assert description in ["gripper canonical pose", "gripper current pose"] # only support canonical pose and current pose for now
        
        # If receptacle_name is given, get the receptacle position and bounding box
        if receptacle_name is not None:
            receptacle_center_position, receptacle_bbox_size = self.get_3d_bbox(receptacle_name)
            receptacle_bbox = np.array([receptacle_center_position - receptacle_bbox_size / 2, 
                                        receptacle_center_position + receptacle_bbox_size / 2])
        
        # If position is given, use it directly, otherwise use grounding model to get the receptacle position
        if position is None:
            object_bbox = self.get_3d_bbox(object_name)
            obstacle_bbox_list = [
                self.get_3d_bbox(obstacle_name) for obstacle_name in self.scene.object_names 
                if obstacle_name not in [object_name, 'table', 'ground', 'cabinet']
            ]
            position = calculate_place_position(
                object_bbox, receptacle_bbox, obstacle_bbox_list, max_tries=100)
        
        # Now we compose the place pose with the position and orientation
        pose = Pose()
        pose.position = Point(*position)
        # remain current orientation
        pose.orientation = self.get_gripper_pose().orientation
        
        return pose

    
    