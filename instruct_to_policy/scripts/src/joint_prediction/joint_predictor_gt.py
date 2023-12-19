import os 
from typing import List, Optional, Union, Tuple
import numpy as np 
import rospy

from joint_prediction.srv import GazeboGetJointsAxes, GazeboGetJointsAxesRequest, GazeboGetJointsAxesResponse
from .joint_prediction_base import JointPredictionBase

class JointPredictionGT(JointPredictionBase):
    
    def __init__(self, service_name="/gazebo/get_joints_axes") -> None:
        # initialize the rosservice proxy
        self.service_name = service_name
        self._get_joint_axes = rospy.ServiceProxy(
            self.service_name,
            GazeboGetJointsAxes
        )
       
    def load_model(self):
        # wait for the service to be ready
        while not rospy.is_shutdown():
            try:
                self._get_joint_axes.wait_for_service(timeout=3.0)
                break
            except rospy.ROSException:
                rospy.loginfo(f"Joint prediction: waiting for {self.service_name} service")

        rospy.loginfo("Joint prediction: remote model service ready")
    
    def predict(self, joint_names):
        """
        Predicts the joint axes for the given list of joint names.

        Args:
            joint_names (list): List of joint names.

        Returns:
            list: List of joint axes.
        """
        # Call /get_joints_axes service and retrieve the joint axes
        request = GazeboGetJointsAxesRequest(joint_names=joint_names)
        response: GazeboGetJointsAxesResponse = self._get_joint_axes(request)
        joints_axes: List[np.ndarray] = response.joints_axes
        
        return joints_axes
