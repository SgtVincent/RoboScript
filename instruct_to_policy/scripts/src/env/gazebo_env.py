from typing import List, Tuple, Dict
import rospy 
from std_srvs.srv import Empty
from gazebo_msgs.srv import (
    GetModelState, 
    GetLinkState,
    GetWorldProperties, 
    GetModelProperties
)
from geometry_msgs.msg import Quaternion, Point, Pose 
from .env import Env
from .gazebo_cameras import GazeboRGBDCameraSet
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
        
        # subscribe to object 3D bounding boxes
        self.gt_bbox_sub = rospy.Subscriber(f"/gazebo_ros_bbox_3d_plugin/bounding_boxes_3d", BoundingBox3DArray, self._gt_bbox_callback)
        self.gazebo_gt_bboxes:List[BoundingBox3D] = None
        
        # services of gazebo
        self.reset_world = rospy.ServiceProxy(f"/{self.node_name}/reset_world", Empty)
        self.reset_simulation = rospy.ServiceProxy(f"/{self.node_name}/reset_simulation", Empty)
        self.get_model_state = rospy.ServiceProxy(f"/{self.node_name}/get_model_state", GetModelState)
        self.get_link_state = rospy.ServiceProxy(f"/{self.node_name}/get_link_state", GetLinkState)
        self.get_world_properties = rospy.ServiceProxy(f"/{self.node_name}/get_world_properties", GetWorldProperties)
        self.get_model_properties = rospy.ServiceProxy(f"/{self.node_name}/get_model_properties", GetModelProperties)

        self.robot_names = ["panda", "fr3", "triple_camera_set"]

        # register camera set
        self.camera_set = GazeboRGBDCameraSet(self.sensor_config['cameras'], 
                                              namespace=self.sensor_config['namespace'], 
                                              sub_pcl=self.sensor_config['gt_point_cloud'])

    def _gt_bbox_callback(self, msg: BoundingBox3DArray):
        """ Callback function for ground truth bounding boxes."""
        self.gazebo_gt_bboxes = msg.bboxes_3d


    def get_obj_names(self)-> List[str]:
        """ Get all object names in the world."""

        objects = [
            obj for obj in self.get_world_properties().model_names
            if obj not in self.robot_names
        ]
        if self.extra_objects:
            objects += self.extra_objects
            
        return objects

    def get_gt_obj_pose(self, obj_name):
        """ Get ground truth object pose from gazebo"""
        # name matching: gazebo model name is different from the name in the world, 
        # but obj_name should be a substring of the gazebo model name
        
        gazebo_model_names = self.get_obj_names()
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
        while self.gazebo_gt_bboxes is None:
            rospy.logdebug("gazebo_env: Waiting for ground truth bounding boxes")
            rospy.sleep(0.5)
            
        # gt bbox of drawer has name: cabinet::drawer0
        if 'drawer' in obj_name:
            object_name = obj_name.replace('.', '::')
        
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

    def get_color(self, obj_name):
        """ Get object color."""
        raise NotImplementedError("get_color() not implemented: Not knowing how to get the ground truth color from gazebo (from mesh file)")

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
            collision_dict["collision"] = self.get_bbox(obj_name)
            collision_dict["type"] = "box"
        except:
            rospy.logwarn(f"Object {obj_name} has no collision mesh or bounding box in gazebo")
            pass

        return collision_dict
