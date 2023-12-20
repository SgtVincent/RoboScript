from typing import List, Tuple, Dict
import copy
import rospy 
from std_srvs.srv import Empty
from gazebo_msgs.srv import (
    GetModelState, 
    GetLinkState,
    GetWorldProperties, 
    GetModelProperties,
    SetModelConfiguration,
    SetModelConfigurationRequest
)
from geometry_msgs.msg import Quaternion, Point, Pose
from .env import Env
from .gazebo_cameras import GazeboRGBDCameraSet
from gazebo_plugins_local.srv import GazeboGetBoundingBoxes
from grasp_detection.msg import BoundingBox3DArray, BoundingBox3D



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

    def get_gt_obj_pose(self, obj_name):
        """ Get ground truth object pose from gazebo"""
        # name matching: gazebo model name is different from the name in the world, 
        # but obj_name should be a substring of the gazebo model name
        
        gazebo_model_names = self.get_gazebo_model_names()
        for gazebo_model_name in gazebo_model_names:
            if obj_name in gazebo_model_name.lower():
                obj_name = gazebo_model_name
                break
        # try query as model 
        resp = self.get_model_state(obj_name, self.frame)
        if resp.success:
            return resp.pose
        
        # try query as link
        link_name = obj_name.replace(".", "::")
        resp = self.get_link_state(link_name, self.frame)
        if resp.success:
            return resp.link_state.pose
        
        # Failed to get state for obj_name
        return None
    
    
    def get_gt_bbox(self, obj_name)->Tuple[List, List]:
        """ Get object bounding box."""
        
        self.gazebo_gt_bboxes:List[BoundingBox3D] = self.get_bounding_boxes().bboxes_3d
            
        # gt bbox of drawer or handle: need to convert to link name
        if 'cabinet.drawer' in obj_name or 'cabinet.handle' in obj_name:
            obj_name = obj_name.replace('.', '::')
        
        for bbox in self.gazebo_gt_bboxes:
            if bbox.object_id == obj_name:
                center = [bbox.center.position.x, bbox.center.position.y, bbox.center.position.z]
                size = [bbox.size.x, bbox.size.y, bbox.size.z]
                return center, size
            
        rospy.logwarn(f"Query object {obj_name} has no ground truth bounding box in gazebo")
        return None, None
    
    def get_sensor_data(self):
        return self.camera_set.get_latest_data()
    
    def get_link_pose(self, link_name, ref_frame="world"):
        """ Get link pose."""

        resp = self.get_link_state(link_name, ref_frame)
        return resp.link_state.pose

    def get_mesh(self, obj_name):
        """ Get object mesh."""
        raise NotImplementedError("get_mesh() not implemented: Not knowing how to get the ground truth mesh from gazebo")

    def get_object_collision(self, obj_name):
        """
        Get object collision mesh/ bounding box, and return as a dictionary
        """
        collision_dict = {}
        try:
            collision_dict["collision"] = self.get_mesh(obj_name)
            collision_dict["type"] = "mesh"
        except:
            pass

        try:
            collision_dict["collision"] = self.get_3d_bbox(obj_name)
            collision_dict["type"] = "box"
        except:
            rospy.logwarn(f"Object {obj_name} has no collision mesh or bounding box in gazebo")
            pass

        return collision_dict
