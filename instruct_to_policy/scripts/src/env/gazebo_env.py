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

    def get_object_pos(self, obj_name):
        """ Get object position."""
        return self.get_model_state(obj_name, self.frame).pose.position
    
    def get_bbox(self, obj_name):
        """ Get object bounding box."""
        # TODO: install the get bounding box plugin in gazebo
        # not available yet 
        return self.get_model_properties(obj_name).bounding_box
    
    def get_color(self, obj_name):
        """ Get object color."""
        raise NotImplementedError("get_color() not implemented: Not knowing how to get the ground truth color from gazebo (from mesh file)")

    def parse_pose(self, object, action="grasp"):
        """ 
        Parse pose of action for the object.
        NOTE: Grounding models/ perception models need to handle this function 
        Currently only use the position of the object and canonical orientation .
        """
        pose = Pose()
        pose.position = self.get_object_pos(object)
        pose.orientation = Quaternion(0., 0., 0., 1.)
        return pose