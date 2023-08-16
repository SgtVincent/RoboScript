import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState, GetWorldProperties, GetModelProperties


class GazeboInterface:
    """ Class to interface with Gazebo."""
    def __init__(self, node_name="gazebo", world_frame="world"):
        self.node_name = node_name
        self.world_frame = world_frame
        self.reset_world = rospy.ServiceProxy(f"/{node_name}/reset_world", Empty)
        self.reset_simulation = rospy.ServiceProxy(f"/{node_name}/reset_simulation", Empty)
        self.get_model_state = rospy.ServiceProxy(f"/{node_name}/get_model_state", GetModelState)
        self.get_world_properties = rospy.ServiceProxy(f"/{node_name}/get_world_properties", GetWorldProperties)
        self.get_model_properties = rospy.ServiceProxy(f"/{node_name}/get_model_properties", GetModelProperties)
