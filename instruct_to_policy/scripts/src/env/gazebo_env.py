from typing import List, Tuple, Dict
import os 
import rospy 
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState, GetWorldProperties, GetModelProperties
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
        self.reset_world = rospy.ServiceProxy(f"/{self.node_name}/reset_world", Empty)
        self.reset_simulation = rospy.ServiceProxy(f"/{self.node_name}/reset_simulation", Empty)
        self.get_model_state = rospy.ServiceProxy(f"/{self.node_name}/get_model_state", GetModelState)
        self.get_world_properties = rospy.ServiceProxy(f"/{self.node_name}/get_world_properties", GetWorldProperties)
        self.get_model_properties = rospy.ServiceProxy(f"/{self.node_name}/get_model_properties", GetModelProperties)

        self.robot_names = ["panda", "fr3"]

    def get_obj_names(self)-> List[str]:
        """ Get all object names in the world."""
        return [
            obj for obj in self.get_world_properties().model_names
            if obj not in self.robot_names
        ]

    def get_obj_pos(self, obj_name):
        """ Get object position."""
        return self.get_model_state(obj_name, self.frame).pose.position
    
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