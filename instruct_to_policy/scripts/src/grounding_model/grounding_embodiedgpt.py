import os 
import numpy as np
import time 
from typing import List, Dict, Tuple
from .grounding_base import GroundingBase

# TODO: import model dependencies 

class GroundingEmbodiedGPT(GroundingBase):
    """
    Grounding environment with model on remote server.
    """
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # TODO: load arguments from kwargs 
        self.model = None
        
    def load_model(self):
        """
        Since the model is on remote server, we need to wait for the model to be ready.
        """
        # TODO: wait for service to be ready 
        while(not self._check_service_ready()):
            time.sleep(1)
            
        # Create prompt template for the query functions.
        self._load_prompt()
    
    def _load_prompt(self):
        """
        Create prompt template for the query functions. 
        """
        # TODO: create prompt template
        pass
    
    def _create_query(self, question: str, image_list: List[np.array], current_detections: Dict)-> str:
        """
        Create query string for the remote service.
        Args:
            question: String, question text
            image_list: List, list of image numpy arrays
            current_detections: dictionary of current object detections {object_name: [bbox]}. Each bbox is [x_min, y_min, x_max, y_max]
        """
        # TODO: create query string with pre-defined prompt template
        pass
    
    def _check_service_ready(self)-> bool:
        """
        Check if the remote service is ready.
        """
        # TODO: check if the remote service is ready
        return True
    
    def _parse_response(self, resp: str, option=""):
        """
        Parse response from the remote service.
        """
        # TODO: parse structured data from the response
        pass
    
    def query_text(self, **kwargs)-> str:
        """
        This function should take in input and return query answer as text.
        """
        # TODO: pseudo code for query text
        image_list = kwargs.get('image_list', None)
        question = kwargs.get('question', None)
        current_detections = kwargs.get('current_detections', None)
        # NOTE by Muyao: We can also pass current object namems in the create_query function 
        full_query = self._create_query(question, image_list, current_detections) 
        resp = self.model.query(full_query)
        resp_text = self._parse_response(resp, option="text")
        return resp_text
        
    def query_detections_dict(self, **kwargs) -> List[List[float]]:
        """
        This function should take in input and return query.
        
        return: A dictionary of object multi-view bounding boxes {object_name: [bbox]}. Each bbox is [x_min, y_min, x_max, y_max]
        
        NOTE 1: @Muyao The bounding box should be in pixel rather than [0, 1] scale. 
        
        """
        # TODO: pseudo code for query 2d bbox
        image_list = kwargs.get('image_list', None)
        question = kwargs.get('question', None)
        full_query = self._create_query(question, image_list)
        self.model.query(full_query)
        resp = self.model.query(full_query)
        resp_detection_dict = self._parse_response(resp, option="bbox")
        return resp_detection_dict
        
    # TODO: add more query functions if needed.
        
        
    
