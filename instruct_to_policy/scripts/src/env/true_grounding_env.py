from typing import List, Tuple, Dict
import rospy 
import json
import numpy as np 
import re 

from geometry_msgs.msg import Quaternion, Pose 
from grasp_detection.msg import Grasp as GraspMsg
from .moveit_gazebo_env import MoveitGazeboEnv
from src.grasp_detection import GraspDetectionBase, GraspDetectionRemote
# from src.grasp_detection.utils import Grasp
from src.env.utils import get_axis_aligned_bbox, pose_msg_to_matrix

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

    def parse_pose(self, object, action="", description="", **kwargs):
        """ 
        Parse pose of action for the object.
        NOTE: Grounding models/ perception models need to handle this function 
        Currently only use the object name to get the grasp pose. All other parameters are ignored.
        """
        
        # special case for drawer handle
        if object in ["drawer", "cabinet"] or "drawer" in object.lower() or "cabinet" in object.lower():
            return self.parse_gt_drawer_handle_grasp_pose(object)
        
        if self.grasp_method == "heuristic":
            pose = self.parse_pose_heuristic(object, action, description)
        elif self.grasp_method == "model":
            pose = self.parse_pose_model(object, action, description)
        else:
            raise NotImplementedError(f"Grasp detection model {self.grasp_method} not implemented in {self.__class__.__name__}")      
        return pose
    
    def parse_grasp_pose(self, object_name, description=""):
        """
        Parse grasp pose for the object. Use ground truth grounding and grasp detection model.
        """
        object_bbox = self.get_3d_bbox(object_name)
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
        
        # sort by score and get the best grasp
        best_grasp_idx = np.argmax(score_list)
        pose = pose_list[best_grasp_idx]
        width = width_list[best_grasp_idx]
        return pose

        
    def parse_pose_default(self, object_name, action="", description=""):
        """
        Default pose parser for the object. Call when all other parsers fail.
        """
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
        # get corresponding handle link ID: cabinet.drawer_0 -> cabinet::link_handle_0
        if 'drawer' in object.lower():
            # get drawer index at the end by regex
            drawer_index = re.findall(r'\d+', object)[-1]
            handle_link = f"cabinet::link_handle_{drawer_index}"
        elif 'cabinet' in object.lower():
            # use drawer_3 by default if no drawer index is specified
            handle_link = "cabinet::link_handle_3"
        else:
            raise NotImplementedError(f"Cannot parse handle pose for object {object}")

        pose = Pose()
        pose.position = self.get_link_pose(handle_link).position 
        pose.orientation = pre_defined_handle_orientation 

        return pose
    
    
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
                
        
            
        
    
# unit test
if __name__ == "__main__":
    # test _load_gt_object_info function 
    cfg = {
        "env": {
            "metadata_files": [
                "/home/junting/franka_ws/src/franka_fisher/instruct_to_policy/data/ycb/metadata.json",
                "/home/junting/franka_ws/src/franka_fisher/instruct_to_policy/data/google_scanned_object/object_metadata.json",
                "/home/junting/franka_ws/src/franka_fisher/instruct_to_policy/data/google_scanned_object/container_metadata.json"
            ]
        }
    }
    object_metadata_files = cfg["env"]["metadata_files"]
    object_info = {}
    for file in object_metadata_files:
        with open(file, 'r') as f:
            metadata = json.load(f)
            for k, v in metadata["objects"].items():
                object_name = v['model_name']
                object_info[object_name] = {
                    'bbox_size': v['bbox_size'],
                    'bbox_center': v['bbox_center'],
                }
    print(object_info)