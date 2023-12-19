import os 
from typing import List, Tuple, Dict
import numpy as np

class JointPredictionBase(object):
    """
    Base class for joint prediction interface
    """
    def __init__(self, **kwargs):
        pass
        
    def load_model(self):
        raise NotImplementedError    
    
    def predict(self, data: Dict):
        raise NotImplementedError
