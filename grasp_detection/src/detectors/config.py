import os 
import numpy as np 
import sys 
# get the path of the current file
current_dir = os.path.dirname(os.path.realpath(__file__))


class ConfigBase:
    def __init__(self):
        self.model_path = ""
        self.model_type = ""
        # grasp detection region of interest (ROI)
        self.resolution = 40
        self.voxel_grid_size = 0.3
        
        self.volume_type = "scalable"
        self.max_gripper_width = 0.08

class ConfigGIGA(ConfigBase):
    def __init__(self):
        super().__init__()
        #TODO: add model path and type when testing GIGA 
        self.qual_th = 0.8
        self.out_th = 0.1

class ConfigAnygrasp(ConfigBase):
    def __init__(self):
        super().__init__()
        self.model_path = os.path.join(current_dir, "anygrasp_sdk", "grasp_detection", "log", "checkpoint_detection.tar")
        self.checkpoint_path = self.model_path
        self.gripper_height = 0.03 # grasp pose depth 
        self.top_down_grasp = False # whether to output top-down grasps
        self.max_grasp_num = 5 # maximum number of grasps to return
        
        