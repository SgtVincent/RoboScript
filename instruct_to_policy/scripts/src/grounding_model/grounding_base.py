import os 
import numpy as np
from typing import List, Dict, Tuple

class GroundingBase(object):
    """
    Base class for grounding model.
    """
    def __init__(self, **kwargs):
        self.model = kwargs.get('model', None)
        self.model_type = kwargs.get('model_type', None)

    def load_model(self):
        """
        This function should initialize/load model with member parameters.
        
        Example: 
        model = GroundingModelXXX()
        model.load_model()
        """
        raise NotImplementedError
    
    def query_text(self, **kwargs)-> str:
        """
        This function should take in input and return query answer as text.
        """
        raise NotImplementedError
    
    def query_2d_bbox_list(self, **kwargs)-> List[List[float]]:
        """
        This function should take in input and return query.
        
        return: a list of 2d bbox in the image frame. List[[x_min, y_min, x_max, y_max]]
        """
        raise NotImplementedError
