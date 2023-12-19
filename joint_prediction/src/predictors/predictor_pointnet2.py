import os 
from typing import List, Optional, Union, Tuple
import numpy as np 
import rospy

from .predictor_base import PredictorBase
from joint_prediction.srv import GetJointsAxes, GetJointsAxesRequest, GetJointsAxesResponse

class PredictorPointnet2(PredictorBase):

    def __init__(self) -> None:
        pass
    
    def load_model(self):
        pass
    
    def predict(self, joint_names):
        pass
