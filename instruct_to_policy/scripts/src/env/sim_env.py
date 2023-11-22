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
from src.env.utils import calculate_place_position, is_collision, adjust_z, pose_msg_to_matrix
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
    
    def parse_grasp_pose(self, object_name, **kwargs):
        """
        Parse grasp pose for the object. Use the grounding model and grasp detection model.
        Args:
            object_name: name of the object
            preferred_position: np.array, prefered position of the gripper
            preferred_direction: np.array, prefered direction of the gripper
            description: str, description of the pose
        """
        preferred_position: np.array = kwargs.get('preferred_position', None)
        preferred_direction:np.array = kwargs.get('preferred_direction', None)
        description:str = kwargs.get('description', None)
        
        bbox_2d_list = self.groudning_model.get_3d_bbox(object_name)
        
        sensor_data = self.get_sensor_data()   
                 
        data = {
            'bbox_2d_list': bbox_2d_list
        }
        data.update(sensor_data)
        
        # call grasp detection service
        pose_list, width_list, score_list = self.grasp_model.predict(data)
        
        rank = []
        if preferred_position is not None:
            # if preferred_position is given, choose the grasp pose closest to the prefered position
            position_list = np.array([np.array([p.position.x, p.position.y, p.position.z]) for p in pose_list])
            distance = np.linalg.norm(position_list - preferred_position, axis=1)
            distance_rank_idx = np.argsort(distance)
            distance_rank = np.zeros(len(distance), dtype=np.int)
            distance_rank[distance_rank_idx] = np.arange(len(distance))
            rank.append(distance_rank)
            
        if preferred_direction is not None:
            # if preferred_direction is given, choose the grasp pose with z-axis closest to the prefered direction
            rotation_list = np.array([pose_msg_to_matrix(p)[:3, :3] for p in pose_list])
            z_axis_list = rotation_list[:, :, 2]
            z_axis_list = z_axis_list / np.linalg.norm(z_axis_list, axis=1, keepdims=True)
            preferred_direction = preferred_direction / np.linalg.norm(preferred_direction)
            cos_similarity = np.sum(z_axis_list * preferred_direction, axis=1)
            cos_similarity_rank_idx = np.argsort(cos_similarity)[::-1]
            cos_similarity_rank = np.zeros(len(cos_similarity), dtype=np.int)
            cos_similarity_rank[cos_similarity_rank_idx] = np.arange(len(cos_similarity))
            rank.append(cos_similarity_rank)
                  
        # sort by score 
        score_rank_idx = np.argsort(score_list)[::-1]
        score_rank = np.zeros(len(score_list), dtype=np.int)
        score_rank[score_rank_idx] = np.arange(len(score_list))
        rank.append(score_rank)

        # get the best grasp based on all the ranks, the grasp with the lowest rank sum is the best grasps
        rank = np.array(rank)
        rank_sum = np.sum(rank, axis=0)
        best_grasp_idx = np.argmin(rank_sum)

        pose = pose_list[best_grasp_idx]
        width = width_list[best_grasp_idx]
        
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
        
        # get the bounding box of the object and all other objectss
        object_bbox = self.get_3d_bbox(object_name)
        object_names = self.get_obj_name_list()
        obstacle_bbox_list = [
            self.get_3d_bbox(obstacle_name) for obstacle_name in object_names 
            if obstacle_name not in [object_name]
        ]
        
        # If receptacle_name is given, get the receptacle position and bounding box
        if receptacle_name is not None:
            receptacle_center_position, receptacle_bbox_size = self.get_3d_bbox(receptacle_name)
            receptacle_bbox = np.array([receptacle_center_position - receptacle_bbox_size / 2, 
                                        receptacle_center_position + receptacle_bbox_size / 2])
        
        # If position is given, use it directly, otherwise use grounding model to get the receptacle position
        if position is None:
            assert receptacle_name is not None, "parse_place_pose: position must be given if receptacle_name is not given"
            position = calculate_place_position(
                object_bbox, receptacle_bbox, obstacle_bbox_list, max_tries=100)
        else:
            # position already given, check if the position is valid, if not, adjust it
            collision_mask = np.array([is_collision(object_bbox, obstacle_bbox) for obstacle_bbox in obstacle_bbox_list])
            # adjust the z position if there is collision
            if np.any(collision_mask):
                collided_bbox_list = np.array(obstacle_bbox_list)[collision_mask]
                position[2] = adjust_z(object_bbox, collided_bbox_list)  
        
        # Now we compose the place pose with the position and orientation
        pose = Pose()
        pose.position = Point(*position)
        # remain current orientation
        pose.orientation = self.get_gripper_pose().orientation
        
        return pose

    
    