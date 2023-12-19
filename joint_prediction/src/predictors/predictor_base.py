import os 
from typing import List, Optional, Union, Tuple
import numpy as np 
import rospy

class PredictorBase:
    
    def __init__(self) -> None:
        pass
    
    def load_model(self):
        raise NotImplementedError
    
    def predict(self, joint_names):
        raise NotImplementedError
    
