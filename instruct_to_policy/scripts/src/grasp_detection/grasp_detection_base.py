import os 
from typing import List, Tuple, Dict
import numpy as np
from grasp_detection.msg import Grasp

class GraspDetectionBase(object):
    """
    Base class for grasp detection models.
    """
    def __init__(self, config: Dict):
        self.config = config
        self.model_path = None
        self.model = None
        
    def load_model(self):
        raise NotImplementedError

    def predict(self, data: Dict)-> List[Grasp]:
        raise NotImplementedError
