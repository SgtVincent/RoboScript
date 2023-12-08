from typing import List, Tuple, Dict
import numpy as np
from numpy.typing import ArrayLike
import shapely
from shapely.geometry import *
from shapely.affinity import *
from geometry_msgs.msg import Quaternion, Pose, Point
from grasp_detection.msg import Grasp 
from src.env.utils import get_axis_aligned_bbox, pose_msg_to_matrix, calculate_place_position, is_collision, adjust_z

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
        # TODO: get the min and max boundary of table from the sim 
        self._min_xy = np.array(self._cfg["env"]["coords"]["bottom_left"])
        self._max_xy = np.array(self._cfg["env"]["coords"]["top_right"])
        self._table_z = self._cfg["env"]["coords"]["table_z"]
        self._range_xy = self._max_xy - self._min_xy

    def get_obj_name_list(self)-> List[str]:
        raise NotImplementedError("get_obj_name_list() not implemented")

    ################# robot contorl interface #################
    def get_gripper_pose(self)->Pose:
        """ return robot gripper pose in robot base frame """
        raise NotImplementedError("get_gripper_pose() not implemented")

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
    
    ################ perception interface ################
    def get_sensor_data(self)->Dict:
        """ get sensor data from perception model """
        raise NotImplementedError("get_sensor_data() not implemented")
    
    def detect_objects(self, **kwargs):
        """ detect objects in the scene with perception model"""
        raise NotImplementedError("detect_objects() not implemented")
    
    def get_object_center_position(self, obj_name)->ArrayLike:
        """ Get object position."""
        raise NotImplementedError("get_object_center_position() not implemented")

    def get_3d_bbox(self, obj_name: str)-> ArrayLike:
        """
        Get the bounding box of the object.
        Args:
            obj_name: name of the object
        Returns:
            bbox: np.ndarray, [x_min, y_min, z_min, x_max, y_max, z_max]
        """
        raise NotImplementedError("get_3d_bbox() not implemented")
    
    
    ################ multimodal grounding interface ################
    def parse_question(self, question, **kwargs):
        """ parse question into a dictionary """
        raise NotImplementedError("parse_question() not implemented")
    
    ################ parse pose tools ############################## 
    def parse_adaptive_shape_grasp_pose(self, object_name, **kwargs)->Pose:
        """
        Parse grasp pose for the object. Use ground truth grounding and grasp detection model.
        Args:
            object_name: name of the object
            preferred_position: Optional(np.array), prefered position of the gripper
            preferred_direction: Optional(np.array), prefered direction of the gripper
            description: str, description of the pose
        """
        object_bbox = self.get_3d_bbox(object_name)
        preferred_position: np.array = kwargs.get('preferred_position', None)
        preferred_direction:np.array = kwargs.get('preferred_direction', None)
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
        grasp_candidates = self.grasp_model.predict(data)
        pose_list = [grasp.grasp_pose for grasp in grasp_candidates]
        width_list = [grasp.grasp_width for grasp in grasp_candidates]
        score_list = [grasp.grasp_score for grasp in grasp_candidates]
    
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

    def parse_place_pose(self, object_name, receptacle_name:str=None, **kwargs)->Pose:
        """
        Parse place pose for the object. Use ground truth grounding and heuristic place position calculation.
        Args:
            object_name: str, name of the object
            receptacle_name: Optional(str), name of the receptacle
            position: Optional(np.array), position of the place pose
            description: Optional(str), description of the pose, "canonical pose" or "current pose"
        """
        # get parameters from kwargs
        position = kwargs.get('position', None)
        if isinstance(position, Point):
            position = np.array([position.x, position.y, position.z])
        if isinstance(position, list):
            position = np.array(position)
        description: str= kwargs.get('description', "current pose") 
        assert description in ["canonical pose", "current pose"] # only support canonical pose and current pose for now
        
        # get the bounding box of the object and all other objectss
        object_bbox = self.get_3d_bbox(object_name)
        object_names = self.get_obj_name_list()
        obstacle_bbox_list = [
            self.get_3d_bbox(obstacle_name) for obstacle_name in object_names 
            if obstacle_name not in [object_name]
        ]
        
        pose = Pose()
        
        # If receptacle_name is given, get the receptacle position and bounding box
        if receptacle_name is not None:
            receptacle_bbox = self.get_3d_bbox(receptacle_name)
            # FIXME: for drawer, just hard code the receptacle position x to [max_x-0.2, max_x]
            if "drawer" in receptacle_name.lower():
                receptacle_bbox[0] = receptacle_bbox[3] - 0.2
        
        # If position is given, use it directly, otherwise use grounding model to get the receptacle position
        if position is None:
            assert receptacle_name is not None, "parse_place_pose: position must be given if receptacle_name is not given"
            position = calculate_place_position(
                object_bbox, receptacle_bbox, obstacle_bbox_list, max_tries=100)
        else:
            # position already given, check if the position is valid, if not, adjust it until no collision found 
            collision_mask = np.array([is_collision(object_bbox, obstacle_bbox) for obstacle_bbox in obstacle_bbox_list])
            # adjust the z position if there is collision
            if np.any(collision_mask):
                collided_bbox_list = np.array(obstacle_bbox_list)[collision_mask]
                position[2] = adjust_z(object_bbox, collided_bbox_list, extra_elevation=0.1)          
            
        pose.position = Point(*position)
        if description == "canonical pose":
            # use canonical orientation
            pose.orientation = Quaternion(-1.0,0.0,0.0,0.0)
        else:
            # remain current orientation
            pose.orientation = self.get_gripper_pose().orientation
        
        return pose

    def parse_central_lift_grasp_pose(self, object_name, description="top")->Pose:
        """
        Parse central lift grasp pose for the object. Use ground truth grounding and heuristic place position calculation.
        """
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

    def parse_horizontal_grasp_pose(self, object)->Pose:
        """ 
        Parse horizontal pose for grasping drawer handle. (master pose for cabinet drawer pulling)
        Currently the position of the handle if the GT bounding box center of handle in gazebo. 
        The gripper is horizontal and perpendicular to the handle.
        """
        pre_defined_horizontal_orientation = Quaternion(-0.5, -0.5, 0.5, 0.5) # for franka hand 
    
        handle_bbox = self.get_3d_bbox(object)
        handle_center = (handle_bbox[:3] + handle_bbox[3:]) / 2
        pose = Pose()
        pose.position = Point(*handle_center)
        pose.orientation = pre_defined_horizontal_orientation 
        return pose