import os 
from typing import List, Tuple, Dict
import numpy as np

class JointPredictionBase(object):
    '''
    Base class for joint prediction interface
    '''
    
    def __init__(self, **kwargs):
        pass
        
    def load_model(self):
        raise NotImplementedError    
    
    def predict(self, data: Dict):
        '''
        This function takes in a dictionary of query data 
        {
            "obj_name": str, # object name
            "joint_types": List[str] # ["revolute", "prismatic"],
            ##### perception data #####
            ....
        }
        
        and returns the predicted joint axes.
        [
            {
                "joint_position":[
                    0.0,
                    0.0,
                    0.0
                ],
                "joint_axis": [
                    -1.0,
                    -8.511809568290118e-08,
                    -1.677630052654422e-07
                ],
                "type": "prismatic"
            },
            ...
        ]
        
        '''
        raise NotImplementedError
