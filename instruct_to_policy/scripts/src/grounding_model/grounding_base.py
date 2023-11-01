import os 
import numpy as np


def GroundingBase(object):
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

    def predict(self, input):
        if self.model_loaded:
            if self.model_type == 'torch':
                return self.model(input)
            elif self.model_type == 'tf':
                return self.model.predict(input)
        else:
            print('Model not loaded')
            return None

    def save_model(self, model_path):
        if self.model_loaded:
            if self.model_type == 'torch':
                torch.save(self.model.state_dict(), model_path)
            elif self.model_type == 'tf':
                self.model.save(model_path)
        else:
            print('Model not loaded')
            return None
