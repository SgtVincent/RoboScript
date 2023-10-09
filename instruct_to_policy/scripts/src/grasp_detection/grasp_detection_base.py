import os 
from typing import List, Tuple, Dict
import numpy as np

class GraspDetectionBase(object):
    """
    Base class for grasp detection models.
    """
    def __init__(self, config: Dict, model_path: str = None):
        self.config = config
        self.model = None
        self.model_path = None

    def load_model(self):
        raise NotImplementedError

    def predict(self, data: Dict):
        raise NotImplementedError

    def predict_batch(self, images):
        # Note: Do we really need this batch prediction function? 
        raise NotImplementedError
