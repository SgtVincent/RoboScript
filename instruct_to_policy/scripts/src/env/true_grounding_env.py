from typing import List, Tuple, Dict
import rospy 
import json
import numpy as np 
import re 

from geometry_msgs.msg import Quaternion, Pose, Point
from grasp_detection.msg import Grasp as GraspMsg
from .moveit_gazebo_env import MoveitGazeboEnv
from src.grasp_detection import GraspDetectionBase, GraspDetectionRemote
# from src.grasp_detection.utils import Grasp
from src.env.utils import get_axis_aligned_bbox, pose_msg_to_matrix, calculate_place_position, is_collision, adjust_z


class TrueGroundingEnv(MoveitGazeboEnv):
    """
    Environment to use gazebo GT model state as observation and grounding.
    """
    def __init__(self, cfg) -> None:
        super().__init__(cfg)
        self.object_metadata_files = cfg["env"]["metadata_files"]
        self.grasp_config = cfg["grasp_detection"]
        self.grasp_method = self.grasp_config["method"] # ["heuristic", "model"]
        
        self.grasp_model = None
        self.groudning_model = None 
        
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
        
    def get_object_center_position(self, obj_name, **kwargs):
        """
        Get the position of the object in the world frame. 
        This function uses ground truth model state from gazebo and ignore all other parameters.
        """
        gt_pose = self.get_gt_obj_pose(obj_name)
        if gt_pose is None:
            return None
        return gt_pose.position
        
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
    
    def parse_grasp_pose(self, object_name, **kwargs):
        """
        Parse grasp pose for the object. Use ground truth grounding and grasp detection model.
        Args:
            object_name: name of the object
            preferred_position: np.array, prefered position of the gripper
            preferred_direction: np.array, prefered direction of the gripper
            description: str, description of the pose
        """
        object_bbox = self.get_3d_bbox(object_name)
        preferred_position: np.array = kwargs.get('preferred_position', None)
        preferred_direction:np.array = kwargs.get('preferred_direction', None)
        description:str = kwargs.get('description', None)
        
        bbox_center = (object_bbox[:3] + object_bbox[3:]) / 2
        bbox_size = object_bbox[3:] - object_bbox[:3]
        
        # get visual input from perception model
        sensor_data = self.get_sensor_data()
            
        # TODO: a model should be able to predict pose for different actions
        data = {
            'bbox_3d':{
                'center': bbox_center,
                'size': bbox_size,
            }
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
        Parse place pose for the object. Use ground truth grounding and heuristic place position calculation.
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

    # TODO: implement this failure recovery behavior if needed
    def parse_pose_default(self, object_name, action="", description=""):
        """
        Default pose parser for the object. Call when all other parsers fail.
        """
        if "drawer" in object_name.lower() or "cabinet" in object_name.lower():
            return self.parse_gt_drawer_handle_grasp_pose(object_name)
        else:
            pose = Pose()
            pose.position = self.get_object_center_position(object_name)
            pose.orientation = Quaternion(0,1,0,0)
            return pose
    
    def parse_gt_drawer_handle_grasp_pose(self, object):
        """ 
        Parse ground truth pose for grasping drawer handle.
        NOTE: Grounding models/ perception models need to handle this function 
        Currently get the position of the handle by get the handle link position from gazebo. 
        The orientation is canonical, perpendicular to the cabinet door.
        """
        pre_defined_handle_orientation = Quaternion(-0.5, -0.5, 0.5, 0.5)
        # pre_defined_gripper_tip_offset = 0.1 # x-axis positive direction
        # get corresponding handle link ID: cabinet.drawer_0 -> cabinet::handle_0
        if 'drawer' in object.lower():
            # get drawer index at the end by regex
            drawer_index = re.findall(r'\d+', object)[-1]
            handle_link = f"cabinet::handle_{drawer_index}"
        elif 'cabinet' in object.lower():
            # use drawer_3 by default if no drawer index is specified
            handle_link = "cabinet::handle_3"
        else:
            raise NotImplementedError(f"Cannot parse handle pose for object {object}")

        pose = Pose()
        pose.position = self.get_link_pose(handle_link).position 
        pose.orientation = pre_defined_handle_orientation 

        return pose
    
    def parse_question(self, question, **kwargs):
        """ parse question into a dictionary """
        raise NotImplementedError("parse_question() not implemented")
    
    def get_lying_objects(self, objects=[], **kwargs):
        """
        Get the list of objects that are lying on the table by their ground truth pose.
        """
        if len(objects) == 0:
            objects = self.get_obj_name_list()

        # judge if the object is lying on the table by its ground truth pose:
        # calculate the angle between the object's z-axis and the table normal
        angle_threnshold = 60 * np.pi / 180 # 60 degree
        table_normal = np.array([0, 0, 1])
        lying_objects = []
        for object in objects:
            object_gt_pose = self.get_gt_obj_pose(object)
            if object_gt_pose is not None:
                object_rotation = pose_msg_to_matrix(object_gt_pose)[:3, :3]
                object_z_axis = object_rotation[:, 2] / np.linalg.norm(object_rotation[:, 2])
                # compute the angle between the object's z-axis and the table normal
                angle = np.arccos(np.dot(object_z_axis, table_normal))

                if angle < angle_threnshold:
                    lying_objects.append(object)
                
        return lying_objects
