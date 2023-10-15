import os 
import numpy as np 

import rospy 
from grasp_detection.srv import DetectGrasps, DetectGraspsRequest, DetectGraspsResponse

class DetectorBase(object):
    def __init__(self, service_name, config):
        self.service_name = service_name
        self.config = config
        
        # initialize ROS service 
        self.service = rospy.Service(self.service_name, DetectGrasps, self.detect_srv_callback)
        rospy.loginfo('DetectGrasps service listener is ready.')

    def detect_callback(self, req: DetectGraspsRequest)->DetectGraspsResponse:
        raise NotImplementedError()
    
    def load_model(self):
        raise NotImplementedError()





