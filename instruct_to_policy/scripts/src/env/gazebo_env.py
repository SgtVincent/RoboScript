from typing import List, Tuple, Dict
import os 
import rospy 
from std_srvs.srv import Empty
from gazebo_msgs.srv import (
    GetModelState, 
    GetLinkState,
    GetWorldProperties, 
    GetModelProperties
)
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Quaternion, Point, Pose 
from .env import Env


class GazeboEnv(Env):
    """ Class to interface with Gazebo."""
    def __init__(self, cfg):
        super().__init__(cfg)
        assert cfg['env']['sim'] == 'gazebo'
        self.node_name = cfg['env']['sim']
        self.frame = cfg['env']['frame']
        self.extra_objects = cfg['env'].get('extra_objects', [])
        self.reset_world = rospy.ServiceProxy(f"/{self.node_name}/reset_world", Empty)
        self.reset_simulation = rospy.ServiceProxy(f"/{self.node_name}/reset_simulation", Empty)
        self.get_model_state = rospy.ServiceProxy(f"/{self.node_name}/get_model_state", GetModelState)
        self.get_link_state = rospy.ServiceProxy(f"/{self.node_name}/get_link_state", GetLinkState)
        self.get_world_properties = rospy.ServiceProxy(f"/{self.node_name}/get_world_properties", GetWorldProperties)
        self.get_model_properties = rospy.ServiceProxy(f"/{self.node_name}/get_model_properties", GetModelProperties)

        self.robot_names = ["panda", "fr3", "triple_camera_set"]

    def get_obj_names(self)-> List[str]:
        """ Get all object names in the world."""

        objects = [
            obj for obj in self.get_world_properties().model_names
            if obj not in self.robot_names
        ]
        if self.extra_objects:
            objects += self.extra_objects
            
        return objects

    def get_obj_pos(self, obj_name):
        """ Get object position."""
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
            return resp.pose.position
        
        # try query as link
        link_name = obj_name.replace(".", "::")
        resp = self.get_link_state(link_name, self.frame)
        if resp.success:
            return resp.link_state.pose.position
        
        # Failed to get state for obj_name
        return None
    
    def get_link_pose(self, link_name, ref_frame="world"):
        """ Get link pose."""

        resp = self.get_link_state(link_name, ref_frame)
        return resp.link_state.pose


    def get_bbox(self, obj_name):
        """ Get object bounding box."""
        # TODO: install the get bounding box plugin in gazebo
        # not available yet 
        return self.get_model_properties(obj_name).bounding_box
    
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

    def parse_pose(self, object, action="", description=""):
        """ 
        Parse pose of action for the object.
        NOTE: Grounding models/ perception models need to handle this function 
        Currently only use the position of the object and canonical orientation .
        """
        # special case for drawer handle
        if object in ["drawer", "cabinet"] or "drawer" in object.lower() or "cabinet" in object.lower():
            return self.parse_drawer_handle_pose(object="cabinet_1076", action=action, description=description)

        if action in ['pick', 'grasp', 'grab', 'get', 'take', 'hold']:
            pose = Pose()
            pose.position = self.get_obj_pos(object)
            if hasattr(self, 'reset_pose'):
                pose.orientation = self.reset_pose.orientation
            else:
                pose.orientation = Quaternion(0,0,0,1)
        elif action in ['place', 'put', 'drop', 'release']:
            pose = Pose()
            pose.position = self.get_obj_pos(object)
            pose.position.z += 0.2
            if hasattr(self, 'reset_pose'):
                pose.orientation = self.reset_pose.orientation
            else:
                pose.orientation = Quaternion(0,0,0,1)
        else:
            pose = Pose()
            pose.position = self.get_obj_pos(object)
            pose.orientation = Quaternion(0,0,0,1)
            if hasattr(self, 'reset_pose'):
                pose.orientation = self.reset_pose.orientation
            else:
                pose.orientation = Quaternion(0,0,0,1)
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