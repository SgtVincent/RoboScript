import os
from typing import List, Tuple, Dict
from typing import Dict 
import numpy as np 
import trimesh 
import rospy 
from cv_bridge import CvBridge


from grasp_detection.srv import DetectGrasps, DetectGraspsRequest, DetectGraspsResponse
from grasp_detection.msg import Grasp, Perception, PerceptionSingleCamera, BoundingBox3D
from geometry_msgs.msg import Pose, Point, Quaternion

from src.perception.utils import data_to_percetion_msg
from .grasp_detection_base import GraspDetectionBase


class GraspDetectionRemote(GraspDetectionBase):
    """
    Wrapper class grasp detection interface: 
    Call service to send all sensor data and perception results to grasp detection node
    """
    def __init__(self, config, **kwargs):
        super().__init__(config)
            
        self.service_name = self.config["service_name"]
        
        # initialize ROS service proxy 
        self.detect_grasps = rospy.ServiceProxy(self.service_name, DetectGrasps)
        self.cv_bridge = CvBridge()
        
        
    def load_model(self):
        # waiting for DetectGrasp service to be ready 
        n_trials = 3
        while n_trials > 0:
            try:
                rospy.wait_for_service(self.service_name, timeout=5)
                break
            except rospy.ROSException:
                n_trials -= 1
                rospy.logwarn(f"Grasp detection: remote model service not ready, {n_trials} trials left")
        if n_trials <= 0:
            raise rospy.ROSException(f"Grasp detection: remote model service not ready after {n_trials} trials")
        rospy.loginfo("Grasp detection: remote model service ready")
    
    def predict(self, data: Dict)-> List[Grasp]:

        perception_msg = data_to_percetion_msg(data, self.cv_bridge)
        request = DetectGraspsRequest(perception_data=perception_msg)
        rospy.loginfo("Sending perception data to grasp detection service")
        response: DetectGraspsResponse = self.detect_grasps(request)
        grasps: List[Grasp] = response.grasps

        return grasps
        
        
    
        
        

