from typing import List, Tuple, Dict
import rospy 
import json
import numpy as np 
from geometry_msgs.msg import Quaternion, Pose 

from grasp_detection.msg import Grasp as GraspMsg
from .moveit_gazebo_env import MoveitGazeboEnv
from src.grasp_detection import GraspDetectionBase, GraspDetectionRemote
from src.grounding_model import GroundingBase, GroundingEmbodiedGPT
# from src.grasp_detection.utils import Grasp
from src.env.utils import get_axis_aligned_bbox, pose_msg_to_matrix

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
        
        self.grounding_model = GroundingEmbodiedGPT(self.grounding_config)


        
    def get_object_center_position(self, obj_name, **kwargs):
        """
        Get the position of the object in the world frame. 
        This function uses ground truth model state from gazebo and ignore all other parameters.
        """
        obj_pose = self.grounding_model.query_2d_bbox_list(obj_name)
        if obj_pose is None:
            return None
        return np.array([obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z])

        
    def get_3d_bbox(self, obj_name, **kwargs):
        """
        Get the 3D bounding box of the object in the world frame.
        This function uses ground truth model state from gazebo and ignore all other parameters.
        """
        center, size = self.get_gt_bbox(obj_name)
        if center is None or size is None:
            return None, None
        return np.array(center), np.array(size)
    
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

    def parse_place_pose(self, object_name, receptacle_name, description="gripper canonical pose"):
        """
        object_name --[MM]--> bbox_2d_list --[3D fusion] --> object_3d_bbox
        receptable_name --[MM]--> bbox_2d_list --[3D fusion] --> receptacle_3d_bbox
        object_3d_bbox, receptacle_3d_bbox --[place_model]--> pose_list --> pose
        """
        # TODO: implement this function
        pass

    
    def parse_drawer_handle_pose(self, object, action="", description=""):
        """ 
        Parse pose of action for the drawer handle.
        NOTE: Grounding models/ perception models need to handle this function 
        Currently get the position of the handle by get the handle link position from gazebo. 
        The orientation is canonical, perpendicular to the cabinet door.
        """
        # TODO: get the orientation of the handle from perception model or find another way to get the orientation
        """
        orientation: 
        x: 0.6714430184317122
        y: 0.26515792374322233
        z: -0.6502306735376142
        w: -0.2367606801525871
        """
        pre_defined_handle_orientation = Quaternion(0.6714430184317122, 0.26515792374322233, -0.6502306735376142, -0.2367606801525871)
        pre_defined_gripper_tip_offset = 0.1 # x-axis positive direction
        # get corresponding handle link ID: cabinet.drawer_0 -> cabinet::link_handle_0
        if 'drawer' in object.lower():
            # get drawer index 
            drawer_index = object.split("_")[-1]
            handle_link = object.split(".")[0] + f"::link_handle_{drawer_index}"
        elif 'cabinet' in object.lower():
            # use drawer_3 by default if no drawer index is specified
            handle_link = object + "::link_handle_3"
        else:
            raise NotImplementedError(f"Cannot parse handle pose for object {object}")

        if action in ['grasp', 'grab'] or 'grasp' in action or 'grab' in action:
            pose = Pose()
            pose.position = self.get_link_pose(handle_link).position 
            pose.position.x += pre_defined_gripper_tip_offset
            pose.orientation = pre_defined_handle_orientation 
        elif action in ['pull', 'open'] or 'pull' in action or 'open' in action:
            
            pose = Pose() 
            pose.position = self.get_link_pose(handle_link).position
            pose.position.x += 0.2 + pre_defined_gripper_tip_offset
            pose.orientation = pre_defined_handle_orientation
        elif  action in ['push', 'close'] or 'push' in action or 'close' in action:
            pose = Pose()
            pose.position = self.get_link_pose(handle_link).position
            pose.position.x -= 0.2 + pre_defined_gripper_tip_offset
            pose.orientation = pre_defined_handle_orientation
        return pose
    
    
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