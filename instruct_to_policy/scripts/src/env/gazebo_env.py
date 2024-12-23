from typing import Any, List, Tuple, Dict
import numpy as np
import rospy 
from std_srvs.srv import Empty
from gazebo_msgs.srv import (
    GetModelState, 
    GetLinkState,
    GetWorldProperties, 
    GetModelProperties,
    GetJointProperties,
    SetModelConfiguration,
    SetModelConfigurationRequest
)
from geometry_msgs.msg import Quaternion, Point, Pose
from gazebo_plugins_local.srv import GazeboGetBoundingBoxes
from grasp_detection.msg import BoundingBox3DArray, BoundingBox3D
from .env import Env
from .gazebo_cameras import GazeboRGBDCameraSet


class GazeboEnv(Env):
    """ Class to interface with Gazebo."""
    def __init__(self, cfg):
        super().__init__(cfg)
        assert cfg['env']['sim'] == 'gazebo'
        self.node_name = cfg['env']['sim']
        self.frame = cfg['env']['frame']
        self.extra_objects = cfg['env'].get('extra_objects', [])
        self.sensor_config = cfg['env']['sensor']
        
        self.gazebo_gt_bboxes:List[BoundingBox3D] = None
        
        # services of gazebo
        self.reset_world = rospy.ServiceProxy(f"/{self.node_name}/reset_world", Empty)
        self.reset_simulation = rospy.ServiceProxy(f"/{self.node_name}/reset_simulation", Empty)
        self.get_model_state = rospy.ServiceProxy(f"/{self.node_name}/get_model_state", GetModelState)
        self.get_link_state = rospy.ServiceProxy(f"/{self.node_name}/get_link_state", GetLinkState)
        self.get_world_properties = rospy.ServiceProxy(f"/{self.node_name}/get_world_properties", GetWorldProperties)
        self.get_model_properties = rospy.ServiceProxy(f"/{self.node_name}/get_model_properties", GetModelProperties)
        self.get_joint_properties = rospy.ServiceProxy(f"/{self.node_name}/get_joint_properties", GetJointProperties)
        self.get_bounding_boxes = rospy.ServiceProxy(f"/{self.node_name}/get_bounding_boxes", GazeboGetBoundingBoxes)

        self.robot_names = ["panda", "fr3", "ur5"]
        self.environment_names = ["ground_plane", "sun", "triple_camera_set"]
        
        # register camera set
        self.camera_set = GazeboRGBDCameraSet(self.sensor_config['cameras'], 
                                              namespace=self.sensor_config['namespace'], 
                                              sub_pcl=self.sensor_config['gt_point_cloud'])
        
        # FIXME: This should be in a config file rather than hard code 
        # NOTE: Hack to reset cabinet joint state, since reset_world does not reset joint state
        # record cabinet joint states for reset
        self.cabinet_joint_init_states = {
            "cabinet::joint_0": 0.0,
            "cabinet::joint_1": 0.0,
            "cabinet::joint_2": 0.0,
            "cabinet::joint_3": 0.0,
        }


    def reset_gazebo(self):
        """
        Reset world state and cabinet joint state.
        """
        rospy.wait_for_service(f"/{self.node_name}/reset_world")
        rospy.wait_for_service(f"/{self.node_name}/reset_simulation")
        rospy.wait_for_service(f"/{self.node_name}/set_model_configuration")
        
        # reset gazebo world state
        self.reset_world()
        
        # reset cabinet joint state
        self.set_joint_positions('cabinet', list(self.cabinet_joint_init_states.keys()), list(self.cabinet_joint_init_states.values()))
        
    
    def set_joint_positions(self, model_name: str, joint_names: List[str], joint_values: List[float]):
        """ Set joint positions"""

        set_model_config = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        config_request = SetModelConfigurationRequest()
        config_request = SetModelConfigurationRequest()
        config_request.model_name = model_name
        # Assuming 'joint_name' is the name of the joint you want to reset
        config_request.joint_names = joint_names
        config_request.joint_positions = joint_values
        
        set_model_config(config_request)
        
    def get_gazebo_model_names(self)-> List[str]:
        """ Get all object names in the world."""

        objects = [
            obj for obj in self.get_world_properties().model_names
            if obj not in self.robot_names and obj not in self.environment_names
        ]
        if self.extra_objects:
            objects += self.extra_objects
            
        return objects

    def get_gt_obj_pose(self, object_name)->Pose:
        """ Get ground truth object pose from gazebo"""
        # name matching: gazebo model name is different from the name in the world, 
        # but object_name should be a substring of the gazebo model name
        
        gazebo_model_names = self.get_gazebo_model_names()
        for gazebo_model_name in gazebo_model_names:
            if object_name in gazebo_model_name.lower():
                object_name = gazebo_model_name
                break
        # try query as model 
        resp = self.get_model_state(object_name, self.frame)
        if resp.success:
            return resp.pose
        
        # try query as link
        link_name = object_name.replace(".", "::")
        resp = self.get_link_state(link_name, self.frame)
        if resp.success:
            return resp.link_state.pose
        
        # Failed to get state for object_name
        return None
    
    
    def get_gt_bbox(self, object_name)->Tuple[np.ndarray, np.ndarray]:
        """ Get object bounding box."""
        
        self.gazebo_gt_bboxes:List[BoundingBox3D] = self.get_bounding_boxes().bboxes_3d
            
        # gt bbox of drawer or handle: need to convert to link name
        if 'cabinet.drawer' in object_name or 'cabinet.handle' in object_name:
            object_name = object_name.replace('.', '::')
        
        for bbox in self.gazebo_gt_bboxes:
            if bbox.object_id == object_name:
                center = np.array([bbox.center.position.x, bbox.center.position.y, bbox.center.position.z])
                size = np.array([bbox.size.x, bbox.size.y, bbox.size.z])
                return center, size
            
        rospy.logwarn(f"Query object {object_name} has no ground truth bounding box in gazebo")
        return None, None
    
    def get_gt_bboxes(self)->Dict[str, Tuple[np.ndarray, np.ndarray]]:
        """ Get ground truth bounding boxes for all objects"""
        
        self.gazebo_gt_bboxes:List[BoundingBox3D] = self.get_bounding_boxes().bboxes_3d
        
        bboxes = {}
        for bbox_3d in self.gazebo_gt_bboxes:
            
            object_name = bbox_3d.object_id
            if 'cabinet::drawer' in object_name or 'cabinet::handle' in object_name:
                object_name = object_name.replace('::', '.')
            center = np.array([bbox_3d.center.position.x, bbox_3d.center.position.y, bbox_3d.center.position.z])
            size = np.array([bbox_3d.size.x, bbox_3d.size.y, bbox_3d.size.z])
            
            # FIXME: enlarge handle bbox to include part of the wooden board
            # TODO: Should do this modification in model definition or external annotation metadata file  
            # enlarge the handle bbox in -x direction for 0.01 m
            # if 'cabinet.handle' in object_name:
            #     center[0] -= 0.01
            #     size[0] += 0.02
            bboxes[object_name] = (center, size)
            
        return bboxes
    
    def get_sensor_data(self) -> Dict[str, Any]:
        return self.camera_set.get_latest_data()
    
    def get_link_pose(self, link_name, ref_frame="world"):
        """ Get link pose."""

        resp = self.get_link_state(link_name, ref_frame)
        return resp.link_state.pose

    def get_mesh(self, object_name):
        """ Get object mesh."""
        raise NotImplementedError("get_mesh() not implemented: Not knowing how to get the ground truth mesh from gazebo")

    def get_object_collision(self, object_name):
        """
        Get object collision mesh/ bounding box, and return as a dictionary
        """
        collision_dict = {}
        try:
            collision_dict["collision"] = self.get_mesh(object_name)
            collision_dict["type"] = "mesh"
        except:
            pass

        try:
            collision_dict["collision"] = self.get_3d_bbox(object_name)
            collision_dict["type"] = "box"
        except:
            rospy.logwarn(f"Object {object_name} has no collision mesh or bounding box in gazebo")
            pass

        return collision_dict
