from .grasp_detection_base import GraspDetectionBase
from .grasp_detection_remote import GraspDetectionRemote

def create_grasp_model(config)->GraspDetectionBase:
    """
    Create grasp detection model from config.
    """
    model_name = config["model_name"]
    
    if model_name in ['anygrasp', 'GIGA']:    
        grasp_model_config = config["model_params"]
        grasp_model = GraspDetectionRemote(grasp_model_config)
        grasp_model.load_model()
        
        return grasp_model
    else:
        raise ValueError("Invalid grasp detection model: {}".format(model_name))