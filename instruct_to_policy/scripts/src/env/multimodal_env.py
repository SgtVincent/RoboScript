import dis
from typing import List, Tuple, Dict
import rospy 
import json
import numpy as np 
from geometry_msgs.msg import Quaternion, Pose, Point

from grasp_detection.msg import Grasp
from .moveit_gazebo_env import MoveitGazeboEnv
from src.grasp_detection import GraspDetectionBase, GraspDetectionRemote
from src.grounding_model import create_grounding_model, GroundingBase
from src.joint_prediction import JointPredictionBase
from src.plane_detection import PlaneDetectionOpen3D
from src.env.utils import (
    calculate_place_position, 
    is_collision, 
    adjust_z, 
    pose_msg_to_matrix, 
    create_collision_object_from_open3d_mesh
)

from src.perception.scene_manager import SceneManager

class MultiModalEnv(MoveitGazeboEnv):
    """
    Simple grounding environment to use gazebo GT model state as observation, and GIGA for grasp pose prediction.
    """
    def __init__(self, cfg) -> None:
        super().__init__(cfg)
        self.use_gt_perception = False 
        self.object_metadata_files = cfg["env"]["metadata_files"]
        self.grasp_config = cfg["grasp_detection"]
        self.grasp_method = self.grasp_config["method"] # ["heuristic", "model"]
        self.plane_detection_config = cfg["plane_detection"]
        
        
        self.grounding_config = cfg["grounding_model"]
        # use glip as default baseline 
        self.grounding_model_name = self.grounding_config.get("model_name", "glip")
        self.grounding_model_args = self.grounding_config.get("model_args", {})        
        
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
        
        self.grounding_model = create_grounding_model(self.grounding_model_name, **self.grounding_model_args)
        # TODO: implement joint prediction model
        self.joint_prediction_model = JointPredictionBase()
        self.plane_detection_model = PlaneDetectionOpen3D(model_params=self.plane_detection_config)
        

    def detect_objects(self, **kwargs):
        """
        Call the perception pipeline to detect all task-specific objects in the scene.
        """
        # get objects to detect
        object_list = kwargs.get('object_list', [])
        # call detection pipeline
        sensor_data = self.get_sensor_data()
        detections_list = self.grounding_model.query_2d_bbox_list(
            sensor_data=sensor_data,
            object_list=object_list
        )
        # update scene 
        data = {
            'detections_list': detections_list
        }
        data.update(sensor_data)
        self.scene.update(data)
        
        # update objects in moveit planning scene 
        self.add_scene_objects_to_moveit()

    def get_obj_name_list(self) -> List[str]:
        """
        Get object name list from scene manager
        """
        return self.scene.get_object_names()

    def get_object_center_position(self, obj_name, **kwargs):
        """
        Get the position of the object in the world frame. 
        This function uses ground truth model state from gazebo and ignore all other parameters.
        """
        return self.scene.get_object_center_position(obj_name)
        
    def get_object_pose(self, obj_name, **kwargs):
        # return self.scene.get_object_pose(obj_name)
        raise NotImplementedError
        
    def get_3d_bbox(self, obj_name, **kwargs)->np.array:
        """
        Get the bounding box of the object.
        Args:
            obj_name: name of the object
        Returns:
            bbox: np.ndarray, [x_min, y_min, z_min, x_max, y_max, z_max]
        """
        return self.scene.get_3d_bbox(obj_name)
    
    def get_plane_normal(self, obj_name: str, position: np.ndarray) -> np.ndarray:
        '''
        Get the plane normal of the object at the given position.
        Args:
            obj_name: name of the object
            position: position of the object
        Returns:
            normal: np.ndarray, [x, y, z]
        '''
        # First get the object point cloud from scene manager
        object_point_cloud = self.scene.get_object_cloud(obj_name)

        # Then calculate all planes with plane detection model
        normal_vectors, oboxes = self.plane_detection_model.detect_planes(object_point_cloud)
        
        # Calculate the distance from position to all planes (point-to-plane-distance)
        distances = []
        for i, obox in enumerate(oboxes):
            plane_center = obox.get_center()    
            normal_vector = normal_vectors[i]
            distance = np.abs(np.dot(normal_vector, plane_center - position))
            distances.append(distance)
        
        # Select the plane that is closest to the given position
        min_idx = np.argmin(distances)
        normal = normal_vectors[min_idx]
        return normal
        

    ####################  Moveit planning related functions ####################
    def add_scene_objects_to_moveit(self, **kwargs):
        """Add all objects in the scene to the moveit planning scene."""
        for object_name in self.get_obj_name_list():
            object_mesh = self.scene.get_object_mesh(object_name)
            self.register_object_mesh(object_mesh, object_name)
            if object_name not in self.objects:
                self.objects[object_name] = {}
                
    

    
    