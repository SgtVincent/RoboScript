#!/usr/bin/env python3

import rospy
from grasp_detection.srv import DetectGrasps, DetectGraspsRequest, DetectGraspsResponse
from grasp_detection.msg import Grasp, Perception, PerceptionSingleCamera, BoundingBox2D

# try:
#     from detectors.detector_anygrasp import DetectorAnygrasp
# except:
#     print("Failed to import anygrasp detector. Make sure you have installed the anygrasp package.")
from detectors.detector_anygrasp import DetectorAnygrasp
try:
    from detectors.detector_giga import DetectorGIGA 
except:
    print("Failed to import GIGA detector. Make sure you have installed the vgn package.")

from detectors.config import ConfigAnygrasp, ConfigGIGA


if __name__ == '__main__':
    
    rospy.init_node('grasp_detector_node')

    # Load ROS parameters 
    model_name = rospy.get_param('~model_name', 'anygrasp') 
    service_name = rospy.get_param('~service_name', '/detect_grasps')
    
    # Initialize detector 
    if model_name == "anygrasp":
        detector = DetectorAnygrasp(service_name, ConfigAnygrasp())
        detector.load_model()
    elif model_name == "giga":
        detector = DetectorGIGA(service_name, ConfigGIGA())
        detector.load_model()
    else: 
        raise ValueError(f"Invalid model name: {model_name}")
    
    rospy.spin()
    