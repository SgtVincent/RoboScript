from typing import List, Tuple, Dict
import rospy 
import json
import numpy as np 
from geometry_msgs.msg import Quaternion, Pose 

from grasp_detection.msg import Grasp as GraspMsg
from .moveit_gazebo_env import MoveitGazeboEnv
from src.grasp_detection import GraspDetectionBase, GraspDetectionRemote
# from src.grasp_detection.utils import Grasp
from src.env.utils import get_axis_aligned_bbox, pose_msg_to_matrix

class SimpleGroundingEnv(MoveitGazeboEnv):
    """
    Simple grounding environment to use gazebo GT model state as observation, and GIGA for grasp pose prediction.
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
        
    def get_obj_pos(self, obj_name, **kwargs):
        """
        Get the position of the object in the world frame. 
        This function uses ground truth model state from gazebo and ignore all other parameters.
        """
        gt_pose = self.get_gt_obj_pose(obj_name)
        if gt_pose is None:
            return None
        return gt_pose.position
        
    def get_bbox(self, obj_name, **kwargs):
        """
        Get the 3D bounding box of the object in the world frame.
        This function uses ground truth model state from gazebo and ignore all other parameters.
        """
        center, size = self.get_gt_bbox(obj_name)
        if center is None or size is None:
            return None, None
        return np.array(center), np.array(size)

    def parse_pose(self, object, action="", description="", **kwargs):
        """ 
        Parse pose of action for the object.
        NOTE: Grounding models/ perception models need to handle this function 
        Currently only use the object name to get the grasp pose. All other parameters are ignored.
        """
        
        # special case for drawer handle
        if object in ["drawer", "cabinet"] or "drawer" in object.lower() or "cabinet" in object.lower():
            return self.parse_drawer_handle_pose(object, action=action, description=description)
        
        if self.grasp_method == "heuristic":
            pose = self.parse_pose_heuristic(object, action, description)
        elif self.grasp_method == "model":
            pose = self.parse_pose_model(object, action, description)
        else:
            raise NotImplementedError(f"Grasp detection model {self.grasp_method} not implemented in {self.__class__.__name__}")      
        return pose
    
    def parse_pose_model(self, object, action="", description=""):
        
        # get axis-aligned bounding box of the object in the world: 
        object_pose = self.get_gt_obj_pose(object)
        # object_transform = pose_msg_to_matrix(object_pose)
        # bbox_center, bbox_size = get_axis_aligned_bbox(self.object_info[object]['bbox_center'], 
        #                                                self.object_info[object]['bbox_size'], object_transform)
        # NOTE: use ground truth bounding box for now
        bbox_center, bbox_size = self.get_bbox(object)
        
        # get visual input from perception model
        sensor_data = self.get_sensor_data()
        
        # get grasp pose from grasp detection model
        if action in ['pick', 'grasp', 'grab', 'get', 'take', 'hold']:
            
            # TODO: a model should be able to predict pose for different actions
            data = {
                'bbox_3d':{
                    'center': bbox_center,
                    'size': bbox_size,
                },
                'object_pose': object_pose,
                'action': action, # NOTE: currently not used
            }
            data.update(sensor_data)
            
            # call grasp detection service
            pose_list, width_list, score_list = self.grasp_model.predict(data)
            
            # sort by score and get the best grasp
            best_grasp_idx = np.argmax(score_list)
            pose = pose_list[best_grasp_idx]
            width = width_list[best_grasp_idx]
            return pose

        else:
            # TODO: how to predict pose for actions other than grasp?
            return self.parse_pose_heuristic(object, action, description)
        
    def parse_pose_heuristic(self, object, action="", description=""):

        if action in ['pick', 'grasp', 'grab', 'get', 'take', 'hold']:
            pose = Pose()
            pose.position = self.get_obj_pos(object)
            if hasattr(self, 'reset_pose'):
                pose.orientation = self.reset_pose.orientation
            else:
                pose.orientation = Quaternion(0,1,0,0)
        elif action in ['place', 'put', 'drop', 'release']:
            pose = Pose()
            pose.position = self.get_obj_pos(object)
            pose.position.z += 0.2
            if hasattr(self, 'reset_pose'):
                pose.orientation = self.reset_pose.orientation
            else:
                pose.orientation = Quaternion(0,1,0,0)
        else:
            rospy.logwarn(f"Action {action} not supported in heuristic grasp model, use default pose")
            pose = Pose()
            pose.position = self.get_obj_pos(object)
            pose.orientation = Quaternion(0,1,0,0)
            if hasattr(self, 'reset_pose'):
                pose.orientation = self.reset_pose.orientation
            else:
                pose.orientation = Quaternion(0,1,0,0)
        return pose
    
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