import os 
from typing import List, Optional, Union, Tuple, Dict
import numpy as np 
import rospy

from std_msgs.msg import String
from gazebo_msgs.srv import GetModelProperties, GetModelPropertiesRequest, GetModelPropertiesResponse
from gazebo_plugins_local.srv import GazeboGetJointsAxes, GazeboGetJointsAxesRequest, GazeboGetJointsAxesResponse
from joint_prediction.msg import JointAxis
from .joint_prediction_base import JointPredictionBase

joint_type_map = {
    1: "revolute",
    3: "prismatic",
}

class JointPredictionGT(JointPredictionBase):
    '''
    Joint prediction using ground truth joint axes from Gazebo. 
    '''
    
    def __init__(self, service_name="/gazebo/get_joints_axes", model_properties_service="/gazebo/get_model_properties") -> None:
        # initialize the rosservice proxy
        self.service_name = service_name
        self.model_properties_service = model_properties_service
        
        self._get_model_properties = rospy.ServiceProxy(
            self.model_properties_service,
            GetModelProperties
        )
        
        self._get_joint_axes = rospy.ServiceProxy(
            self.service_name,
            GazeboGetJointsAxes
        )


    def load_model(self):
        # wait for all services to be ready in while loop
        service_ready_model_properties = False
        service_ready_joint_axes = False
        
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service(self.service_name, timeout=1.0)
                service_ready_joint_axes = True
            except rospy.ROSException as e:
                rospy.logwarn(f"Joint prediction: remote model service {self.service_name} not ready, retrying...")
                continue
            
            try:
                rospy.wait_for_service(self.model_properties_service, timeout=1.0)
                service_ready_model_properties = True
            except rospy.ROSException as e:
                rospy.logwarn(f"Joint prediction: remote model service {self.model_properties_service} not ready, retrying...")
                continue
                
            if service_ready_model_properties and service_ready_joint_axes:
                break

        rospy.loginfo("Joint prediction: all remote model services ready!")

    
    def get_model_joint_names(self, object_name: str)->List[str]:
        """
        Get the list of joint names of the object from Gazebo
        """
        request = GetModelPropertiesRequest(model_name=object_name)
        response: GetModelPropertiesResponse = self._get_model_properties(request)
        return response.joint_names
    
    
    def get_joint_axes(self, object_name: str)->List[JointAxis]:
        """
        Get the list of joint axes of the object from Gazebo
        """
        joint_names = self.get_model_joint_names(object_name)
        full_joint_names = [String(f"{object_name}::{joint_name}") for joint_name in joint_names]
        request = GazeboGetJointsAxesRequest(joint_names=full_joint_names)
        response: GazeboGetJointsAxesResponse = self._get_joint_axes(request)
        return response.joints_axes
    
    def predict(self, data: Dict)->List[Dict]:
        """
        Predicts the joint axes for the given object. Given a model name and list of types,
        - first get the list of joint names by calling /gazebo/get_model_properties service
        - then call /gazebo/get_joints_axes service to get the all joints axes

        Args:
            data: Dict, query data
            {
                "object_name": str,
                "joint_types": List[str] # ["revolute", "prismatic"],
            }

        Returns:
            list: List of dictionaries of joint axes.
            [
                {
                    "joint_position":np.array([
                        0.0,
                        0.0,
                        0.0
                    ]),
                    "joint_axis": np.array([
                        -1.0,
                        -8.511809568290118e-08,
                        -1.677630052654422e-07
                    ]),
                    "type": "prismatic"
                },  
                ...
            ]  
        """
        # Call /get_joints_axes service and retrieve the joint axes
        joints_axes:List[JointAxis] = self.get_joint_axes(data["object_name"])

        # convert the list of JointAxis to a list of dictionary 
        ret = []
        for joint_axis in joints_axes:
            
            # filter out the joint axes that are not in the given joint types
            if joint_axis.type not in joint_type_map or joint_type_map[joint_axis.type] not in data["joint_types"]:
                continue
            
            joint_axis_dict = {
                "joint_position": np.array([
                    joint_axis.origin.x,
                    joint_axis.origin.y,
                    joint_axis.origin.z,
                ]),
                "joint_axis": np.array([
                    joint_axis.axis.x,
                    joint_axis.axis.y,
                    joint_axis.axis.z,
                ]),
                "type": joint_type_map[joint_axis.type],
            }
            ret.append(joint_axis_dict)
            
        return ret
