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
        self.voxel_size = 0.003
        # maximum number of grasps to return
        self.max_grasp_num = 10
        self.volume_type = "scalable"
        self.color_type = "rgb"
        
        # self.max_gripper_width = 0.08 # franka hand 
        self.max_gripper_width = 0.1 # robotiq 85
        # environment-specific parameters
        self.table_height = 0.82 # height of the table plane

        # random seed 
        self.random_seed = 42


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
        self.gripper_height = 0.05 # grasp pose depth 
        self.top_down_grasp = False # whether to output top-down grasps
        # NOTE: by default should be True, otherwise grasps outside of the region of interest (ROI) will be predicted first 
        # This could lead to no valid grasp 
        self.filter_cloud_with_bbox = True # whether to filter point cloud with 2D or 3D bbox
        # add margin to the 2D and 3D bbox to filter the point cloud, to increase grasp detection range 
        self.filter_bbox_2d_margin = 5 # in pixel
        self.filter_bbox_3d_margin = 0.15 # in meter
        self.filter_table_plane = False # whether to filter table plane
        
        # safety config: 
        # bound size limit for the input point cloud
        self.bound_size_limit = np.array([0.6, 0.6, 0.6])  
        # max number of points in the input point cloud
        self.max_point_num = 100000 
        
        self.debug = False # whether to visualize the grasps 

        
        
        